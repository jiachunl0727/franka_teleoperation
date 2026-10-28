// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include "safety_filter/cbf_types.hpp"

namespace safety_filter::cbf {
struct SingularityMeasure {
  double value{0};  // det(J_scaled J_scaled^T), squared dimensionless manipulability
  Vector7d gradient{Vector7d::Zero()};
  double velocity_curvature{0};  // dq^T Hessian(value) dq
  bool valid{false};
};
// Seven serial revolute joints. Perturb the measured instantaneous screw axes,
// retaining actual backend/tool transforms; no hard-coded Panda forward model.
// Fixed central differences of exact local product-of-exponentials kinematics.
SingularityMeasure singularity_measure(const ArmState& state, double characteristic_length,
                                       bool derivatives = true) noexcept;
}  // namespace safety_filter::cbf
