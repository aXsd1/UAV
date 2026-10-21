#include "vtol_dynamics/model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace vtol
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double deg(double rad) { return rad * 180.0 / kPi; }
double rad(double deg) { return deg * kPi / 180.0; }

Vec3 cross(const Vec3 &a, const Vec3 &b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
void add(Vec3 &a, const Vec3 &b)
{
  for (int i = 0; i < 3; ++i) a[i] += b[i];
}
}  // namespace

// ---------------------------------------------------------------------------
Table Table::load(const std::string &path)
{
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  Table t;
  std::string line;
  std::vector<std::string> header;
  std::getline(f, line);
  {
    std::stringstream ss(line);
    std::string h;
    while (std::getline(ss, h, ',')) {
      if (!h.empty() && h.back() == '\r') h.pop_back();
      header.push_back(h);
      t.cols_[h];
    }
  }
  while (std::getline(f, line)) {
    if (line.empty() || line == "\r") continue;
    std::stringstream ss(line);
    std::string v;
    for (const auto &h : header) {
      if (!std::getline(ss, v, ',')) throw std::runtime_error("short row in " + path);
      t.cols_[h].push_back(std::stod(v));
    }
  }
  return t;
}

const std::vector<double> &Table::col(const std::string &name) const
{
  auto it = cols_.find(name);
  if (it == cols_.end()) throw std::runtime_error("no column " + name);
  return it->second;
}

double Table::interp(const std::string &x, const std::string &y, double at) const
{
  const auto &xs = col(x);
  const auto &ys = col(y);
  if (at <= xs.front()) return ys.front();
  if (at >= xs.back()) return ys.back();
  const auto hi = std::upper_bound(xs.begin(), xs.end(), at) - xs.begin();
  const auto lo = hi - 1;
  const double t = (at - xs[lo]) / (xs[hi] - xs[lo]);
  return ys[lo] + t * (ys[hi] - ys[lo]);
}

// ---------------------------------------------------------------------------
Model::Model(const std::string &dir)
{
  std::ifstream f(dir + "/aero.json");
  if (!f) throw std::runtime_error("cannot open " + dir + "/aero.json");
  const auto j = nlohmann::json::parse(f);

  p_.S = j["reference"]["area_m2"];
  p_.b = j["reference"]["span_m"];
  p_.c = j["reference"]["chord_m"];
  p_.s_ref = j["reference"]["moment_reference_station_m"];
  p_.s_cg = j["mass"]["cg_station_m"];
  p_.mass = j["mass"]["mass_kg"];
  p_.rho = j["air_density_kg_m3"];
  p_.D = j["rotor"]["diameter_m"];
  p_.I_rotor = j["rotor"]["rotor_inertia_kg_m2"];
  p_.motor_tau = j["rotor"]["motor_time_constant_s"];
  p_.sigma_a = double(j["duct"]["exit_area_ratio"]) * double(j["duct"]["disk_area_m2"]);
  p_.s_duct = j["duct"]["center_station_m"];
  p_.s_vane = j["duct"]["exit_station_m"];
  p_.vane_max = rad(j["vanes"]["max_deflection_deg"]);
  p_.vane_rate = rad(j["vanes"]["servo_rate_deg_s"]);
  p_.vane_deadband = rad(j["vanes"]["deadband_deg"]);
  p_.elevon_max = rad(j["elevons"]["max_deflection_deg"]);
  p_.hover_rpm = j["hover_allocation"]["rpm"];
  for (const auto &[k, v] : j["derivatives_per_rad"].items()) p_.deriv[k] = v;

  alpha_ = Table::load(dir + "/airframe_alpha.csv");
  beta_ = Table::load(dir + "/airframe_beta.csv");
  rotor_ = Table::load(dir + "/rotor.csv");
  vane_ = Table::load(dir + "/vane.csv");
}

std::pair<double, double> Model::rotor(double omega_rad_s, double v_axial) const
{
  const double n = omega_rad_s / (2.0 * kPi);
  if (n < 1e-3) return {0.0, 0.0};
  const double j = std::max(v_axial, 0.0) / (n * p_.D);
  const double ct = rotor_.interp("J", "CT", j);
  const double cq = rotor_.interp("J", "CQ", j);
  const double D2 = p_.D * p_.D;
  return {ct * p_.rho * n * n * D2 * D2, cq * p_.rho * n * n * D2 * D2 * p_.D};
}

Output Model::wrench(const Inputs &in) const
{
  Output out;
  const auto &dv = p_.deriv;
  const double u = in.v_air[0];

  const auto [t_up, q_up] = rotor(in.rotor_speed[0], u);
  const auto [t_lo, q_lo] = rotor(in.rotor_speed[1], u);
  const double thrust = t_up + t_lo;

  double mdot = 0.0, ve = std::max(u, 0.0);
  if (thrust > 0.0) {
    const double uu = std::max(u, 0.0);
    ve = 0.5 * (uu + std::sqrt(uu * uu + 4.0 * thrust / (p_.rho * p_.sigma_a)));
    mdot = p_.rho * p_.sigma_a * ve;
  }
  const double jet = mdot * ve;

  Vec3 F{}, M{};

  // Rotors; upper CCW seen from the nose -> reaction torque -Q about +x.
  const double dp = deg(in.vane[0]), dy = deg(in.vane[1]);
  const double side_p = vane_.interp("delta_deg", "side_force_ratio", dp);
  const double side_y = vane_.interp("delta_deg", "side_force_ratio", dy);
  const double loss = vane_.interp("delta_deg", "axial_loss_ratio", dp) +
                      vane_.interp("delta_deg", "axial_loss_ratio", dy);
  F[0] += thrust - jet * loss;
  M[0] += q_lo - q_up;

  const Vec3 h{p_.I_rotor * (in.rotor_speed[0] - in.rotor_speed[1]), 0.0, 0.0};
  const Vec3 gyro = cross(in.omega, h);
  for (int i = 0; i < 3; ++i) M[i] -= gyro[i];
  M[0] -= p_.I_rotor * (in.rotor_accel[0] - in.rotor_accel[1]);

  // Exit vanes.
  const Vec3 f_vane{0.0, jet * side_y, -jet * side_p};
  add(F, f_vane);
  add(M, cross(arm(p_.s_vane), f_vane));

  // Duct momentum drag.
  const Vec3 f_md{0.0, -mdot * in.v_air[1], -mdot * in.v_air[2]};
  add(F, f_md);
  add(M, cross(arm(p_.s_duct), f_md));

  // Airframe.
  const double V = std::sqrt(in.v_air[0] * in.v_air[0] + in.v_air[1] * in.v_air[1] + in.v_air[2] * in.v_air[2]);
  if (V >= 1e-3) {
    const double alpha = std::atan2(in.v_air[2], in.v_air[0]);
    const double beta = std::asin(std::clamp(in.v_air[1] / V, -1.0, 1.0));
    const double ad = deg(alpha), bd = deg(beta);
    double cl = alpha_.interp("alpha_deg", "CL", ad);
    const double cd = alpha_.interp("alpha_deg", "CD", ad);
    double cm = alpha_.interp("alpha_deg", "Cm", ad);
    const double cy = beta_.interp("beta_deg", "CY", bd);
    double c_roll = beta_.interp("beta_deg", "Cl", bd);
    const double cn = beta_.interp("beta_deg", "Cn", bd);

    const double de = 0.5 * (in.elevon[0] + in.elevon[1]);
    const double da = 0.5 * (in.elevon[0] - in.elevon[1]);
    cl += dv.at("CL_elevon_sym") * de;
    cm += dv.at("Cm_elevon_sym") * de;
    c_roll += dv.at("Cl_elevon_diff") * da;

    const double qbar = 0.5 * p_.rho * V * V;
    // The alpha tables only see the x-z plane flow (q cos^2 beta); see model.py.
    const double qbar_xz = 0.5 * p_.rho * (in.v_air[0] * in.v_air[0] + in.v_air[2] * in.v_air[2]);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    const double k = 0.25 * p_.rho * V * p_.S;
    const Vec3 f_af{qbar_xz * p_.S * (cl * sa - cd * ca),
                    qbar * p_.S * cy + k * p_.b * dv.at("CYr") * in.omega[2],
                    qbar_xz * p_.S * (-cl * ca - cd * sa) - k * p_.c * dv.at("CLq") * in.omega[1]};
    const double b2 = p_.b * p_.b, c2 = p_.c * p_.c;
    const Vec3 m_ref{
        qbar * p_.S * p_.b * c_roll + k * b2 * (dv.at("Clp") * in.omega[0] + dv.at("Clr") * in.omega[2]),
        qbar_xz * p_.S * p_.c * cm + k * c2 * dv.at("Cmq") * in.omega[1],
        qbar * p_.S * p_.b * cn + k * b2 * (dv.at("Cnr") * in.omega[2] + dv.at("Cnp") * in.omega[0])};
    add(F, f_af);
    add(M, m_ref);
    add(M, cross(arm(p_.s_ref), f_af));
    out.alpha = alpha;
    out.beta = beta;
  }

  out.force = F;
  out.moment = M;
  out.thrust = thrust;
  out.mdot = mdot;
  out.v_exit = ve;
  return out;
}

// ---------------------------------------------------------------------------
Pair ActuatorState::step(const Params &p, const Pair &cmd_rotor, const Pair &cmd_vane, const Pair &cmd_elevon,
                         double dt)
{
  Pair accel{};
  const double a = 1.0 - std::exp(-dt / p.motor_tau);
  for (int i = 0; i < 2; ++i) {
    const double prev = rotor[i];
    rotor[i] += (cmd_rotor[i] - rotor[i]) * a;
    accel[i] = (rotor[i] - prev) / dt;
  }
  const double rate = p.vane_rate * dt;
  auto servo = [&](double cur, double cmd, double limit) {
    double err = std::clamp(cmd, -limit, limit) - cur;
    if (std::abs(err) < p.vane_deadband) err = 0.0;
    return cur + std::clamp(err, -rate, rate);
  };
  for (int i = 0; i < 2; ++i) {
    vane[i] = servo(vane[i], cmd_vane[i], p.vane_max);
    elevon[i] = servo(elevon[i], cmd_elevon[i], p.elevon_max);
  }
  return accel;
}

}  // namespace vtol
