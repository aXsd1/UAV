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

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rotor_cmd = motor_cmd_;
      vane_cmd = {servo_cmd_[0], servo_cmd_[1]};
      elevon_cmd = {servo_cmd_[2], servo_cmd_[3]};
      elevator_cmd = servo_cmd_[4];
      wind = wind_;
    }
    const Pair accel = actuators_.step(vehicle_->params(), rotor_cmd, vane_cmd, elevon_cmd, dt, elevator_cmd);

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
};

}  // namespace vtol

GZ_ADD_PLUGIN(vtol::VtolDynamics, gz::sim::System, vtol::VtolDynamics::ISystemConfigure,
              vtol::VtolDynamics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(vtol::VtolDynamics, "vtol::VtolDynamics")
