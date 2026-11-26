#include "teleop_controllers/inverse_dynamics_controller.hpp"

namespace teleop_controllers {
bool InverseDynamicsParameters::valid() const noexcept {
  return common.valid() && stiffness.allFinite() && (stiffness.array() > 0).all() &&
         std::isfinite(damping_ratio) && damping_ratio > 0 && nullspace_stiffness.allFinite() &&
         (nullspace_stiffness.array() >= 0).all() && nullspace_damping.allFinite() &&
         (nullspace_damping.array() >= 0).all();
}

NominalTorque InverseDynamicsController::compute(
    const ArmState& s, const ControlReference& reference, double) noexcept {
  const auto* target = std::get_if<CartesianReference>(&reference);
  if (!target || !target->position.allFinite() || !target->orientation.coeffs().allFinite() ||
      std::abs(target->orientation.norm() - 1.) > .01 || !target->twist.allFinite() ||
      !target->acceleration.allFinite())
    return {Vector7::Zero(), StateFault::invalid_reference};
  if (!s.mass.allFinite() || !s.mass.isApprox(s.mass.transpose(), 1e-10) || !s.q.allFinite() ||
      !s.dq.allFinite() || !s.coriolis.allFinite() || !s.position.allFinite() ||
      !s.orientation.coeffs().allFinite() || std::abs(s.orientation.norm() - 1.) > .01 ||
      !s.jacobian.allFinite())
    return {Vector7::Zero(), StateFault::nonfinite_state};
  const Eigen::LLT<Eigen::Matrix<double, 7, 7>> mass(s.mass);
  if (mass.info() != Eigen::Success)
    return {Vector7::Zero(), StateFault::nonfinite_state};
  const Eigen::Matrix<double, 7, 6> minv_jt = mass.solve(s.jacobian.transpose());
  const Matrix6 lambda_inverse = s.jacobian * minv_jt;
  const Eigen::LDLT<Matrix6> task(lambda_inverse);
  if (task.info() != Eigen::Success || !task.isPositive() || task.rcond() < 1e-10)
    return {Vector7::Zero(), StateFault::singular_task};
  const Matrix6 lambda = task.solve(Matrix6::Identity());
  Matrix6 damping;
  if (!modal_damping(lambda, p_.stiffness, p_.damping_ratio, damping))
    return {Vector7::Zero(), StateFault::singular_task};

  Vector6 error;
  error.head<3>() = s.position - target->position;
  const Eigen::Matrix3d current = s.orientation.toRotationMatrix();
  const Eigen::Matrix3d desired = target->orientation.toRotationMatrix();
  // OSCBF orientation_error_3D, expressed in the base frame. This is a sine
  // error, not the legacy controller's quaternion logarithm.
  error.tail<3>() =
      -.5 * (current.col(0).cross(desired.col(0)) + current.col(1).cross(desired.col(1)) +
             current.col(2).cross(desired.col(2)));
  const Vector6 velocity_error = s.jacobian * s.dq - target->twist;
  // Matrix-valued OSCBF acceleration gains preserve physical wrench stiffness:
  // Kp = Lambda^-1 K, Kd = Lambda^-1 D. No Jdot*dq term: same law as OSCBF.
  const Matrix6 kp = lambda_inverse * p_.stiffness.asDiagonal();
  const Matrix6 kd = lambda_inverse * damping;
  const Vector6 acceleration = target->acceleration - kp * error - kd * velocity_error;
  Vector6 wrench = lambda * acceleration;
  if (p_.common.cartesian_wrench_limits_enabled) {
    const double force = wrench.head<3>().norm(), moment = wrench.tail<3>().norm();
    if (force > p_.common.cartesian_force_limit)
      wrench.head<3>() *= p_.common.cartesian_force_limit / force;
    if (moment > p_.common.cartesian_moment_limit)
      wrench.tail<3>() *= p_.common.cartesian_moment_limit / moment;
  }
  Vector7 torque = s.jacobian.transpose() * wrench + s.coriolis;
  const Jacobian jbar_transpose = lambda * minv_jt.transpose();
  const Eigen::Matrix<double, 7, 7> nt =
      Eigen::Matrix<double, 7, 7>::Identity() - s.jacobian.transpose() * jbar_transpose;
  const Vector7 joint_acceleration =
      p_.nullspace_stiffness.cwiseProduct(p_.common.home_joint_position - s.q) -
      p_.nullspace_damping.cwiseProduct(s.dq);
  torque.noalias() += nt * s.mass * joint_acceleration;
  if (!torque.allFinite())
    return {Vector7::Zero(), StateFault::nonfinite_nominal};
  // The shared owner applies magnitude/rate clipping and optional CBF verification.
  return {torque, StateFault::none};
}
}  // namespace teleop_controllers
