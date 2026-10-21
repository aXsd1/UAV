// Checks the C++ force model against the Python reference (test/vectors.json).
//   test_model <aero_dir> <vectors.json>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <nlohmann/json.hpp>

#include "vtol_dynamics/model.hpp"

int main(int argc, char **argv)
{
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <aero_dir> <vectors.json>\n", argv[0]);
    return 2;
  }
  vtol::Model model(argv[1]);
  std::ifstream f(argv[2]);
  const auto cases = nlohmann::json::parse(f);

  int failed = 0;
  for (const auto &c : cases) {
    model.set_cg_station(c["cg_station"]);
    const auto &in = c["inputs"];
    vtol::Inputs x;
    for (int i = 0; i < 3; ++i) {
      x.v_air[i] = in["v_air"][i];
      x.omega[i] = in["omega"][i];
    }
    for (int i = 0; i < 2; ++i) {
      x.rotor_speed[i] = in["rotor_speed"][i];
      x.vane[i] = in["vane"][i];
      x.elevon[i] = in["elevon"][i];
      x.rotor_accel[i] = in["rotor_accel"][i];
    }
    x.elevator = in["elevator"];
    const auto out = model.wrench(x);
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double ef = c["force"][i], em = c["moment"][i];
      worst = std::max(worst, std::abs(out.force[i] - ef) / (1e-6 + std::max(1.0, std::abs(ef))));
      worst = std::max(worst, std::abs(out.moment[i] - em) / (1e-6 + std::max(1.0, std::abs(em))));
    }
    const bool ok = worst < 1e-9;
    failed += !ok;
    std::printf("%-28s %s (max rel err %.2e)\n", c["name"].get<std::string>().c_str(), ok ? "ok" : "FAIL", worst);
  }
  std::printf("%zu cases, %d failed\n", cases.size(), failed);
  return failed ? 1 : 0;
}
