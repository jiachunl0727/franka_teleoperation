#pragma once
#include "cbf_safety_filter/constraints.hpp"
namespace cbf_safety_filter {
// Bounded work; anchor must satisfy every supplied row and actuator interval.
// Projects worsening torque components, then certifies a segment back to anchor.
// This experimental directional filter is not a minimum-deviation QP solver.
Vec7 recovery_projection(const ConstraintSet&, const Vec7& nominal, const TorqueBounds&,
                         const Vec7& weights, int sweeps) noexcept;
}  // namespace cbf_safety_filter
