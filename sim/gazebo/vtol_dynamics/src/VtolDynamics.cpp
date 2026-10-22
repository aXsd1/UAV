// Gazebo Harmonic system: applies the ducted tail-sitter force model to the vehicle link.
//
// SDF:
//   <plugin filename="VtolDynamics" name="vtol::VtolDynamics">
//     <aero_dir>/path/to/models/aero/p1</aero_dir>
//     <link_name>base_link</link_name>
//     <wind>0 0 0</wind>                  <!-- mean wind, world ENU, m/s -->
//     <motor_upper>0</motor_upper> <motor_lower>1</motor_lower>   <!-- indices in command/motor_speed -->
//     <servo_vane_pitch>0</servo_vane_pitch> <servo_vane_yaw>1</servo_vane_yaw>
//     <servo_elevon_left>2</servo_elevon_left> <servo_elevon_right>3</servo_elevon_right>
//     <servo_elevator>4</servo_elevator>
//   </plugin>
//
// The link must follow PX4's tailsitter convention: base_link is the multicopter
// frame (FLU, +z along the thrust axis / nose, -x towards the fixed-wing lift
// direction), with its origin at the centre of mass.
// Fixed-wing FRD <-> link FLU:  flu = (fw.z, -fw.y, fw.x),  fw = (flu.z, -flu.y, flu.x).
// Inputs:  /<model>/command/motor_speed (gz.msgs.Actuators, rad/s)
//          /model/<model>/servo_<i>     (gz.msgs.Double, rad)
//          /model/<model>/wind          (gz.msgs.Vector3d, world ENU m/s, optional override)
//          /model/<model>/turbulence    (gz.msgs.Double, gust sigma m/s per axis, Gauss-Markov, tau 1.5 s)
//          /model/<model>/failure       (gz.msgs.StringMsg: "none", "motor <0|1>", "servo <0..4> stuck")
// Output:  .../link/airspeed_link/sensor/air_speed/air_speed (gz.msgs.AirSpeed), the topic PX4's gz_bridge
//          reads. Published here because Gazebo's own air_speed sensor does not see this plugin's wind
//          (it reported ground speed as airspeed in every windy run).

#include <array>
#include <cmath>
#include <random>
#include <sstream>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/air_speed.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include "vtol_dynamics/model.hpp"

namespace vtol
{
namespace
{
gz::math::Vector3d fw_to_flu(const Vec3 &v) { return {v[2], -v[1], v[0]}; }
Vec3 flu_to_fw(const gz::math::Vector3d &v) { return {v.Z(), -v.Y(), v.X()}; }
}  // namespace

class VtolDynamics : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    const std::string model_name = model_.Name(ecm);
    const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;
    link_ = gz::sim::Link(model_.LinkByName(ecm, link_name));
    if (!link_.Valid(ecm)) {
      gzerr << "[VtolDynamics] link '" << link_name << "' not found\n";
      return;
    }
    link_.EnableVelocityChecks(ecm, true);

    const std::string dir = sdf->Get<std::string>("aero_dir", "").first;
    try {
      vehicle_ = std::make_unique<Model>(dir);
    } catch (const std::exception &e) {
      gzerr << "[VtolDynamics] " << e.what() << "\n";
      return;
    }
    wind_ = sdf->Get<gz::math::Vector3d>("wind", gz::math::Vector3d::Zero).first;
    motor_idx_ = {sdf->Get<int>("motor_upper", 0).first, sdf->Get<int>("motor_lower", 1).first};
    servo_idx_ = {sdf->Get<int>("servo_vane_pitch", 0).first, sdf->Get<int>("servo_vane_yaw", 1).first,
                  sdf->Get<int>("servo_elevon_left", 2).first, sdf->Get<int>("servo_elevon_right", 3).first,
                  sdf->Get<int>("servo_elevator", 4).first};

    node_.Subscribe("/" + model_name + "/command/motor_speed", &VtolDynamics::OnMotors, this);
    for (int k = 0; k < 5; ++k) {
      const int idx = servo_idx_[k];
      std::function<void(const gz::msgs::Double &)> cb = [this, k](const gz::msgs::Double &m) {
        std::lock_guard<std::mutex> lock(mutex_);
        servo_cmd_[k] = m.data();
      };
      node_.Subscribe("/model/" + model_name + "/servo_" + std::to_string(idx), cb);
    }
    std::function<void(const gz::msgs::Vector3d &)> wind_cb = [this](const gz::msgs::Vector3d &m) {
      std::lock_guard<std::mutex> lock(mutex_);
      wind_ = {m.x(), m.y(), m.z()};
    };
    node_.Subscribe("/model/" + model_name + "/wind", wind_cb);
    std::function<void(const gz::msgs::Double &)> turb_cb = [this](const gz::msgs::Double &m) {
      std::lock_guard<std::mutex> lock(mutex_);
      turbulence_sigma_ = std::max(0.0, m.data());
    };
    node_.Subscribe("/model/" + model_name + "/turbulence", turb_cb);
    std::function<void(const gz::msgs::StringMsg &)> fail_cb = [this](const gz::msgs::StringMsg &m) {
      std::lock_guard<std::mutex> lock(mutex_);
      std::istringstream in(m.data());
      std::string what;
      int idx = -1;
      in >> what >> idx;
      if (what == "none") {
        motor_failed_ = {false, false};
        servo_stuck_.fill(false);
      } else if (what == "motor" && idx >= 0 && idx < 2) {
        motor_failed_[idx] = true;
      } else if (what == "servo" && idx >= 0 && idx < 5) {
        servo_stuck_[idx] = true;
        stuck_pending_[idx] = true;  // latch the current deflection on the next step
      }
      gzmsg << "[VtolDynamics] failure: " << m.data() << "\n";
    };
    node_.Subscribe("/model/" + model_name + "/failure", fail_cb);
    rng_.seed(sdf->Get<unsigned>("seed", 1u).first);
    const std::string world_name = gz::sim::World(gz::sim::worldEntity(ecm)).Name(ecm).value_or("default");
    airspeed_pub_ = node_.Advertise<gz::msgs::AirSpeed>("/world/" + world_name + "/model/" + model_name +
                                                        "/link/airspeed_link/sensor/air_speed/air_speed");
    gzmsg << "[VtolDynamics] loaded " << dir << " for " << model_name << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || !vehicle_) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0.0) return;

    Pair rotor_cmd{}, vane_cmd{}, elevon_cmd{};
    double elevator_cmd = 0.0;
    gz::math::Vector3d wind;
    double sigma = 0.0;
    std::array<bool, 5> stuck{}, latch{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rotor_cmd = motor_cmd_;
      for (int i = 0; i < 2; ++i) {
        if (motor_failed_[i]) rotor_cmd[i] = 0.0;
      }
      vane_cmd = {servo_cmd_[0], servo_cmd_[1]};
      elevon_cmd = {servo_cmd_[2], servo_cmd_[3]};
      elevator_cmd = servo_cmd_[4];
      wind = wind_;
      sigma = turbulence_sigma_;
      stuck = servo_stuck_;
      latch = stuck_pending_;
      stuck_pending_.fill(false);
    }
    // Stuck servos: hold the deflection they had when the failure started.
    std::array<double *, 5> servo_state{&actuators_.vane[0], &actuators_.vane[1], &actuators_.elevon[0],
                                        &actuators_.elevon[1], &actuators_.elevator};
    for (int k = 0; k < 5; ++k) {
      if (latch[k]) stuck_value_[k] = *servo_state[k];
    }
    const Pair accel = actuators_.step(vehicle_->params(), rotor_cmd, vane_cmd, elevon_cmd, dt, elevator_cmd);
    for (int k = 0; k < 5; ++k) {
      if (stuck[k]) *servo_state[k] = stuck_value_[k];
    }

    // Turbulence: first-order Gauss-Markov gust per axis on top of the mean wind.
    const double a = std::exp(-dt / kGustTau);
    const double b = sigma * std::sqrt(1.0 - a * a);
    for (int i = 0; i < 3; ++i) gust_[i] = a * gust_[i] + b * normal_(rng_);
    wind += gz::math::Vector3d(gust_[0], gust_[1], gust_[2]);

    const auto pose = link_.WorldPose(ecm);
    const auto v_world = link_.WorldLinearVelocity(ecm);
    const auto w_world = link_.WorldAngularVelocity(ecm);
    if (!pose || !v_world || !w_world) return;
    const auto &q = pose->Rot();

    Inputs in;
    in.v_air = flu_to_fw(q.RotateVectorReverse(*v_world - wind));
    in.omega = flu_to_fw(q.RotateVectorReverse(*w_world));
    in.rotor_speed = actuators_.rotor;
    in.vane = actuators_.vane;
    in.elevon = actuators_.elevon;
    in.elevator = actuators_.elevator;
    in.rotor_accel = accel;

    const Output out = vehicle_->wrench(in);

    // Pitot along the fixed-wing nose axis (body x), 20 Hz of sim time.
    const double t_sim = std::chrono::duration<double>(info.simTime).count();
    if (t_sim - last_airspeed_pub_ >= 0.05) {
      last_airspeed_pub_ = t_sim;
      const double u = std::max(0.0, in.v_air[0]);
      gz::msgs::AirSpeed msg;
      msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(t_sim));
      msg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>((t_sim - std::floor(t_sim)) * 1e9));
      msg.set_diff_pressure(0.5 * vehicle_->params().rho * u * u + 0.5 * normal_(rng_));  // Pa, 0.5 Pa noise
      msg.set_temperature(288.15);  // K
      airspeed_pub_.Publish(msg);
    }
    link_.AddWorldWrench(ecm, q.RotateVector(fw_to_flu(out.force)), q.RotateVector(fw_to_flu(out.moment)));
  }

private:
  void OnMotors(const gz::msgs::Actuators &m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 2; ++i) {
      if (motor_idx_[i] < m.velocity_size()) motor_cmd_[i] = std::max(0.0, m.velocity(motor_idx_[i]));
    }
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  std::unique_ptr<Model> vehicle_;
  ActuatorState actuators_;
  gz::transport::Node node_;
  std::mutex mutex_;
  gz::math::Vector3d wind_;
  std::array<int, 2> motor_idx_{0, 1};
  std::array<int, 5> servo_idx_{0, 1, 2, 3, 4};
  Pair motor_cmd_{};
  std::array<double, 5> servo_cmd_{};
  // Scenario hooks (Monte Carlo): turbulence and actuator failures.
  static constexpr double kGustTau = 1.5;  // s
  double turbulence_sigma_{0.0};
  std::array<double, 3> gust_{};
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::array<bool, 2> motor_failed_{};
  std::array<bool, 5> servo_stuck_{}, stuck_pending_{};
  std::array<double, 5> stuck_value_{};
  gz::transport::Node::Publisher airspeed_pub_;
  double last_airspeed_pub_{-1.0};
};

}  // namespace vtol

GZ_ADD_PLUGIN(vtol::VtolDynamics, gz::sim::System, vtol::VtolDynamics::ISystemConfigure,
              vtol::VtolDynamics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vtol::VtolDynamics, "vtol::VtolDynamics")
