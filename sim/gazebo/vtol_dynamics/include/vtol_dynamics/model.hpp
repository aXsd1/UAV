// 6-DOF force model of the ducted tail-sitter, C++ port of tools/dynamics/model.py.
// No Gazebo dependency: the gz system (VtolDynamics.cpp) and the unit test both use it.
// Frame: fixed-wing body FRD (x nose, y right wing, z down); moments about the CG.
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace vtol
{
using Vec3 = std::array<double, 3>;
using Pair = std::array<double, 2>;

/// CSV table with a header row; columns accessed by name.
class Table
{
public:
  static Table load(const std::string &path);
  const std::vector<double> &col(const std::string &name) const;
  /// Linear interpolation of column `y` against column `x`, clamped at the ends (numpy.interp).
  double interp(const std::string &x, const std::string &y, double at) const;

private:
  std::map<std::string, std::vector<double>> cols_;
};

struct Params
{
  double S{}, b{}, c{}, s_ref{}, s_cg{}, rho{};
  double D{}, I_rotor{}, motor_tau{}, sigma_a{}, s_duct{}, s_vane{};
  double vane_max{}, vane_rate{}, vane_deadband{}, elevon_max{};
  double mass{}, hover_rpm{};
  std::map<std::string, double> deriv;
};

struct Inputs
{
  Vec3 v_air{};         ///< body velocity relative to air, FRD, m/s
  Vec3 omega{};         ///< p, q, r, rad/s
  Pair rotor_speed{};   ///< upper, lower, rad/s
  Pair vane{};          ///< pitch, yaw deflection, rad
  Pair elevon{};        ///< left, right, rad, trailing edge down +
  Pair rotor_accel{};   ///< rad/s^2
};

struct Output
{
  Vec3 force{};
  Vec3 moment{};
  double thrust{}, mdot{}, v_exit{}, alpha{}, beta{};
};

class Model
{
public:
  /// Loads aero.json and the CSV tables written by tools/aerogen.
  explicit Model(const std::string &aero_dir);

  Output wrench(const Inputs &in) const;

  const Params &params() const { return p_; }
  void set_cg_station(double s) { p_.s_cg = s; }
  void set_rho(double rho) { p_.rho = rho; }

  /// Thrust (N, along +x) and torque magnitude (N m) of one rotor.
  std::pair<double, double> rotor(double omega_rad_s, double v_axial) const;

private:
  Vec3 arm(double station) const { return {p_.s_cg - station, 0.0, 0.0}; }

  Params p_;
  Table alpha_, beta_, rotor_, vane_;
};

/// First-order motor lag, rate-limited servos with deadband.
struct ActuatorState
{
  Pair rotor{}, vane{}, elevon{};
  /// Advances by dt; returns rotor acceleration (rad/s^2).
  Pair step(const Params &p, const Pair &cmd_rotor, const Pair &cmd_vane, const Pair &cmd_elevon, double dt);
};

}  // namespace vtol
