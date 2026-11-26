#pragma once

#include "teleop_controllers/panda_limits.hpp"
#include "teleop_controllers/torque_controller.hpp"

namespace teleop_controllers {
struct CartesianPdParameters {
  double lambda_translation{50}, lambda_rotation{40};
  double gain_translation{20}, gain_rotation{2.5};
  bool cartesian_wrench_limits_enabled{false};
  double cartesian_force_limit{10.0}, cartesian_moment_limit{1.5};
  Vector7 home_joint_position{Eigen::Map<const Vector7>(teleop_controllers::kHome.data())};
  double nullspace_stiffness{10.0}, nullspace_damping{6.324555320336759};
  double nullspace_joint_limit_gain{2.0}, nullspace_joint_limit_margin{0.35};
  double nullspace_max_torque{5.0}, nullspace_ramp_seconds{0.25};
  bool valid() const noexcept;
};

class CartesianPdController final : public TorqueController {
 public:
  explicit CartesianPdController(const CartesianPdParameters& parameters)
      : p_(parameters) {
    lambda_ << p_.lambda_translation, p_.lambda_translation, p_.lambda_translation,
        p_.lambda_rotation, p_.lambda_rotation, p_.lambda_rotation;
    gain_ << p_.gain_translation, p_.gain_translation, p_.gain_translation, p_.gain_rotation,
        p_.gain_rotation, p_.gain_rotation;
  }
  NominalTorque compute(const ArmState& state, const ControlReference& reference,
                        double period) noexcept override;
  void reset(const ArmState&) noexcept override { elapsed_ = 0; }

 private:
  CartesianPdParameters p_;
  Vector6 lambda_, gain_;
  double elapsed_{0};
};
}  // namespace teleop_controllers
