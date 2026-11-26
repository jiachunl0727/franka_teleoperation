#pragma once

#include "teleop_controllers/cartesian_pd_controller.hpp"

namespace teleop_controllers {
// Cartesian PD about a position captured when orientation mode begins.
// The manager supplies that fixed position and removes translational feedforward.
class OrientationOnlyController final : public TorqueController {
 public:
  explicit OrientationOnlyController(const CartesianPdParameters& parameters)
      : pd_controller_(parameters) {}
  void reset(const ArmState& state) noexcept override { pd_controller_.reset(state); }
  NominalTorque compute(const ArmState& state, const ControlReference& reference,
                        double period) noexcept override;

 private:
  CartesianPdController pd_controller_;
};
}  // namespace teleop_controllers
