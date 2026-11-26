#pragma once

#include "teleop_controllers/control_types.hpp"

namespace teleop_controllers {
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// Simultaneously diagonalize inertia and stiffness by congruence. In modal
// coordinates inertia is I, stiffness is diag(omega^2), damping is 2*zeta*omega.
// Fixed-size storage only; failure never substitutes a different inertia.
inline bool modal_damping(const Matrix6& inertia, const Vector6& stiffness, double damping_ratio,
                          Matrix6& damping) noexcept {
  if (!inertia.allFinite() || !inertia.isApprox(inertia.transpose(), 1e-10) ||
      !stiffness.allFinite() || (stiffness.array() <= 0).any() || !std::isfinite(damping_ratio) ||
      damping_ratio <= 0)
    return false;
  const Eigen::LLT<Matrix6> factor(inertia);
  if (factor.info() != Eigen::Success)
    return false;
  const Matrix6 lower = factor.matrixL();
  const Matrix6 inverse_lower = lower.triangularView<Eigen::Lower>().solve(Matrix6::Identity());
  const Matrix6 whitened = inverse_lower * stiffness.asDiagonal() * inverse_lower.transpose();
  const Eigen::SelfAdjointEigenSolver<Matrix6> eigen(whitened);
  if (eigen.info() != Eigen::Success || (eigen.eigenvalues().array() <= 0).any())
    return false;
  const Matrix6 modes = lower * eigen.eigenvectors();
  damping = modes * (2 * damping_ratio * eigen.eigenvalues().array().sqrt()).matrix().asDiagonal() *
            modes.transpose();
  return damping.allFinite();
}
}  // namespace teleop_controllers
