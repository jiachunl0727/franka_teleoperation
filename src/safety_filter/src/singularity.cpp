// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#include "safety_filter/singularity.hpp"

#include <cmath>
namespace safety_filter::cbf {
namespace {
using Axes = std::array<Eigen::Vector3d, 7>;
Jacobian perturbed(const Axes& origins, const Axes& axes, const Eigen::Vector3d& end,
                   const Vector7d& delta) noexcept {
  Eigen::Affine3d change = Eigen::Affine3d::Identity();
  Axes moved_origins, moved_axes;
  for (int j = 0; j < 7; ++j) {
    moved_origins[j] = change * origins[j];
    moved_axes[j] = change.linear() * axes[j];
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(delta[j], axes[j]).toRotationMatrix();
    Eigen::Affine3d motion = Eigen::Affine3d::Identity();
    motion.linear() = rotation;
    motion.translation() = origins[j] - rotation * origins[j];
    change = change * motion;
  }
  const Eigen::Vector3d position = change * end;
  Jacobian jacobian;
  for (int j = 0; j < 7; ++j) {
    jacobian.col(j).head<3>() = moved_axes[j].cross(position - moved_origins[j]);
    jacobian.col(j).tail<3>() = moved_axes[j];
  }
  return jacobian;
}
}  // namespace
SingularityMeasure singularity_measure(const ArmState& s, double length,
                                       bool derivatives) noexcept {
  SingularityMeasure result;
  if (!std::isfinite(length) || length <= 0 || !s.dq.allFinite())
    return result;
  Axes origins, axes;
  for (int j = 0; j < 7; ++j) {
    origins[j] = s.frame_pose[j].translation();
    axes[j] = s.frame_jacobian[j].col(j).tail<3>();
    if (!origins[j].allFinite() || !axes[j].allFinite() || std::abs(axes[j].norm() - 1) > 1e-6)
      return result;
  }
  const Eigen::Vector3d end = s.frame_pose[8].translation();
  const Jacobian initial = perturbed(origins, axes, end, Vector7d::Zero());
  // Check the assumed chain against the backend's actual end-effector Jacobian.
  if (!end.allFinite() || !s.frame_jacobian[8].allFinite() ||
      !initial.isApprox(s.frame_jacobian[8], 1e-5))
    return result;
  const auto evaluate = [&](const Vector7d& delta) {
    Jacobian j = perturbed(origins, axes, end, delta);
    j.topRows<3>() /= length;
    return (j * j.transpose()).determinant();
  };
  const Vector7d zero = Vector7d::Zero();
  result.value = evaluate(zero);
  // Roundoff at exact rank loss may produce a tiny negative determinant.
  if (!std::isfinite(result.value) || result.value < -1e-9)
    return result;
  result.value = std::max(0.0, result.value);
  if (derivatives) {
    constexpr double step = 1e-4;  // radians; no differentiation across time/reset
    for (int j = 0; j < 7; ++j) {
      Vector7d delta = zero;
      delta[j] = step;
      result.gradient[j] = (evaluate(delta) - evaluate(-delta)) / (2 * step);
    }
    const double speed = s.dq.norm();
    if (speed > 1e-10) {
      const Vector7d delta = (step / speed) * s.dq;
      result.velocity_curvature =
          (evaluate(delta) - 2 * result.value + evaluate(-delta)) * (speed * speed / (step * step));
    }
  }
  result.valid = result.gradient.allFinite() && std::isfinite(result.velocity_curvature);
  return result;
}
}  // namespace safety_filter::cbf
