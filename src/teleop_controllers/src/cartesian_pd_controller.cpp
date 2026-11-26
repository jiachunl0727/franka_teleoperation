#include "teleop_controllers/cartesian_pd_controller.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_controllers {
bool CartesianPdParameters::valid() const noexcept {
  for (double value : {lambda_translation, lambda_rotation, gain_translation, gain_rotation,
                       cartesian_force_limit, cartesian_moment_limit})
    if (!std::isfinite(value) || value <= 0)
      return false;
  for (double value : {nullspace_stiffness, nullspace_damping, nullspace_joint_limit_gain,
                       nullspace_joint_limit_margin, nullspace_max_torque, nullspace_ramp_seconds})
    if (!std::isfinite(value) || value <= 0)
      return false;
  for (int j = 0; j < 7; ++j)
    if (!std::isfinite(home_joint_position[j]) ||
        home_joint_position[j] <= teleop_controllers::kJointLower[j] ||
        home_joint_position[j] >= teleop_controllers::kJointUpper[j] ||
        2 * nullspace_joint_limit_margin >=
            teleop_controllers::kJointUpper[j] - teleop_controllers::kJointLower[j])
      return false;
  return true;
}

NominalTorque CartesianPdController::compute(const ArmState& s,
                                                    const ControlReference& reference,
                                                    double period) noexcept {
  const auto* target = std::get_if<CartesianReference>(&reference);
  if (!target)
    return {Vector7::Zero(), StateFault::invalid_reference};
  Vector6 error = Vector6::Zero();
  error.head<3>() = s.position - target->position;
  Eigen::Quaterniond rotation(target->orientation.conjugate() * s.orientation);
  if (rotation.w() < 0)
    rotation.coeffs() *= -1;
  const Eigen::AngleAxisd angle(rotation);
  error.tail<3>() = target->orientation * (angle.axis() * angle.angle());
  const Vector6 r = s.jacobian * s.dq - target->twist + lambda_.cwiseProduct(error);
  Vector6 wrench = -gain_.cwiseProduct(r);
  if (p_.cartesian_wrench_limits_enabled) {
    const double force = wrench.head<3>().norm(), moment = wrench.tail<3>().norm();
    if (force > p_.cartesian_force_limit)
      wrench.head<3>() *= p_.cartesian_force_limit / force;
    if (moment > p_.cartesian_moment_limit)
      wrench.tail<3>() *= p_.cartesian_moment_limit / moment;
  }
  Vector7 torque = s.jacobian.transpose() * wrench + s.coriolis;
  if (!s.mass.allFinite() || !s.mass.isApprox(s.mass.transpose(), 1e-9))
    return {Vector7::Zero(), StateFault::nonfinite_state};
  const Eigen::LLT<Eigen::Matrix<double, 7, 7>> mass(s.mass);
  if (mass.info() != Eigen::Success)
    return {Vector7::Zero(), StateFault::nonfinite_state};
  // M = L L^T, A = J L^-T. Project in whitened coordinates, then map
  // back to torque: N_tau = L (I - A^+ A) L^-1. No damped inverse:
  // damping an inverse would leak secondary torque into the primary task.
  const Eigen::Matrix<double, 7, 7> lower = mass.matrixL();
  const Jacobian a =
      lower.triangularView<Eigen::Lower>().solve(s.jacobian.transpose()).transpose();
  const Eigen::JacobiSVD<Jacobian> svd(a, Eigen::ComputeFullV);
  Eigen::Matrix<double, 7, 7> kernel = Eigen::Matrix<double, 7, 7>::Identity();
  const double threshold = std::max(1e-12, svd.singularValues()[0] * 1e-8);
  for (int i = 0; i < 6; ++i)
    if (svd.singularValues()[i] > threshold)
      kernel.noalias() -= svd.matrixV().col(i) * svd.matrixV().col(i).transpose();
  Vector7 bias =
      p_.nullspace_stiffness * (p_.home_joint_position - s.q) - p_.nullspace_damping * s.dq;
  for (int j = 0; j < 7; ++j) {
    const double lo =
        (teleop_controllers::kJointLower[j] + p_.nullspace_joint_limit_margin - s.q[j]) /
        p_.nullspace_joint_limit_margin;
    const double hi =
        (s.q[j] - teleop_controllers::kJointUpper[j] + p_.nullspace_joint_limit_margin) /
        p_.nullspace_joint_limit_margin;
    bias[j] +=
        p_.nullspace_joint_limit_gain * (std::clamp(lo, 0.0, 1.0) * std::clamp(lo, 0.0, 1.0) -
                                         std::clamp(hi, 0.0, 1.0) * std::clamp(hi, 0.0, 1.0));
  }
  Vector7 secondary = lower * kernel * lower.triangularView<Eigen::Lower>().solve(bias);
  // Scale the whole vector to retain the projection after applying the cap.
  const double peak = secondary.cwiseAbs().maxCoeff();
  if (peak > p_.nullspace_max_torque)
    secondary *= p_.nullspace_max_torque / peak;
  elapsed_ += period;
  torque += std::clamp(elapsed_ / p_.nullspace_ramp_seconds, 0.0, 1.0) * secondary;
  return {torque, StateFault::none};
}
}  // namespace teleop_controllers
