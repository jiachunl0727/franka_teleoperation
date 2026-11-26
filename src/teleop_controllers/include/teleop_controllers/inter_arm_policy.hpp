#pragma once

#include <cmath>

#include "cbf_safety_filter/kinematics.hpp"
#include "teleop_controllers/robot_model.hpp"

namespace teleop_controllers {
// Data-valid snapshot; constraint violations do not make geometry invalid.
inline bool valid_collision_snapshot(const cbf_safety_filter::PeerSnapshot& p) noexcept {
  if (!p.valid || !p.timestamp_ns || p.count <= 0 || p.count > cbf_safety_filter::kMaxPoints ||
      !std::isfinite(p.speed_limit) || !std::isfinite(p.acceleration_limit) || p.speed_limit <= 0 ||
      p.acceleration_limit <= 0)
    return false;
  for (int i = 0; i < p.count; ++i)
    if (!p.spheres[i].position.allFinite() || !p.spheres[i].velocity.allFinite() ||
        !p.spheres[i].span.allFinite() || !std::isfinite(p.spheres[i].radius) ||
        p.spheres[i].radius <= 0)
      return false;
  return true;
}
// Startup observation only. Its real acquisition timestamp is retained; current
// peer age/skew/domain checks still decide whether the first torque is allowed.
inline cbf_safety_filter::PeerSnapshot stationary_collision_seed(
    const cbf_safety_filter::ModelView& model,
    const cbf_safety_filter::Configuration& config) noexcept {
  auto seed = cbf_safety_filter::collision_snapshot(model, config);
  seed.valid &= model.timestamp_ns != 0 && model.dq.cwiseAbs().maxCoeff() <= .02;
  return seed;
}

// Invoke after the connection barrier, immediately before requesting control.
// Read must acquire a NEW state; the clock never freshens cached data.
// Hardware evaluation also validates Idle/no-current-errors. Work is non-RT.
template <class Read, class Clock, class Evaluate>
cbf_safety_filter::PeerSnapshot acquire_stationary_collision_seed(
    Read&& read, Clock&& clock, Evaluate&& evaluate,
    const cbf_safety_filter::Configuration& config) {
  const auto observation = read();
  const uint64_t acquired_ns = clock();
  const auto model = evaluate(observation);
  const cbf_safety_filter::ModelView view{model.q,        model.dq,         model.mass,
                                          model.coriolis, model.frame_pose, model.frame_jacobian,
                                          acquired_ns};
  return stationary_collision_seed(view, config);
}

// Strict unchanged-law check. The owner may separately accept a certified
// local-only limit while retaining the pose anchor; inter-arm-driven changes
// still stop the pair rather than deliberately yielding the released arm.
inline bool preserves_held_arm(const cbf_safety_filter::Vec7& clipped_hold,
                               const cbf_safety_filter::Vec7& candidate,
                               double tolerance) noexcept {
  return clipped_hold.allFinite() && candidate.allFinite() &&
         (candidate - clipped_hold).cwiseAbs().maxCoeff() <= tolerance;
}
}  // namespace teleop_controllers
