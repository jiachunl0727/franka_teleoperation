#pragma once

#include "teleop_controllers/cartesian_pd_controller.hpp"
#include "teleop_controllers/modal_damping.hpp"

namespace teleop_controllers {
struct InverseDynamicsParameters {
  CartesianPdParameters common;
  Vector6 stiffness{(Vector6() << 750, 750, 750, 75, 75, 75).finished()};
  double damping_ratio{1.0};
  // Acceleration-feedback gains, as in OSCBF's nullspace joint task.
  Vector7 nullspace_stiffness{Vector7::Constant(10.0)};
  Vector7 nullspace_damping{Vector7::Constant(6.324555320336759)};
  bool valid() const noexcept;
};

// OSCBF operational-space torque structure, with physical Cartesian stiffness
// and modal damping converted to matrix-valued acceleration feedback gains.
// Gravity is supplied by the Franka interface; no gravity is added here.
class InverseDynamicsController final : public TorqueController {
 public:
  explicit InverseDynamicsController(const InverseDynamicsParameters& parameters)
      : p_(parameters) {}
  NominalTorque compute(const ArmState&, const ControlReference&, double period) noexcept override;

 private:
  InverseDynamicsParameters p_;
};
}  // namespace teleop_controllers
