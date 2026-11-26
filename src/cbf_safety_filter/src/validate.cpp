#include <iostream>

#include "cbf_safety_filter/safety_filter.hpp"
int main(int argc, char** argv) {
  if (argc != 2 && argc != 3)
    return 2;
  try {
    const auto c = cbf_safety_filter::load_configuration(argv[1], argc == 3 ? argv[2] : "");
    // Startup-only selected-backend preflight, before bringup creates any
    // owner/Hand actions. No model computation or hardware operation occurs.
    const cbf_safety_filter::SafetyFilter workspace(c);
    std::cout << (c.recovery_torque_method == cbf_safety_filter::RecoveryTorqueMethod::native_qp
                      ? "recovery_native_qp"
                      : "directional_recovery")
              << " backend=" << cbf_safety_filter::name(c.backend) << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
