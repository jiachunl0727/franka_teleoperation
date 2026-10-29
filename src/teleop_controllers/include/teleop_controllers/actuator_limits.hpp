#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "teleop_controllers/control_types.hpp"
#include "teleop_controllers/panda_limits.hpp"

namespace teleop_controllers {
inline constexpr double kDefaultActuatorLimitScale = .35;

struct ActuatorLimitScales {
  double torque{kDefaultActuatorLimitScale};
  double torque_rate{kDefaultActuatorLimitScale};
  bool valid() const noexcept {
    return std::isfinite(torque) && torque > 0 && torque <= 1 && std::isfinite(torque_rate) &&
           torque_rate > 0 && torque_rate <= 1;
  }
};

// Scale the published FCI ceilings once, not already derated software limits.
// Configuration validates scales before calling these helpers; no ROS or I/O.
inline Vector7 panda_torque_limits(double scale = kDefaultActuatorLimitScale) noexcept {
  return scale * Eigen::Map<const Vector7>(kJointTorque.data());
}
inline double panda_torque_rate_limit(double scale = kDefaultActuatorLimitScale) noexcept {
  return scale * kJointTorqueRate;
}
}  // namespace teleop_controllers

namespace teleop_controllers {
struct ActuatorBounds {
  Vector7 lower, upper;
  Vector7 clamp(const Vector7& requested) const noexcept {
    return requested.cwiseMax(lower).cwiseMin(upper);
  }
};

// Exact intersection of command magnitude and rate intervals. No solver,
// scheduling, fallback policy or hardware operation is implied by this helper.
inline std::optional<ActuatorBounds> actuator_bounds(const Vector7& previous,
                                                     const Vector7& magnitude, double rate,
                                                     double period) noexcept {
  if (!previous.allFinite() || !magnitude.allFinite() || (magnitude.array() <= 0).any() ||
      !std::isfinite(rate) || rate <= 0 || !std::isfinite(period) || period <= 0 || period > .02)
    return std::nullopt;
  const double delta = rate * std::min(period, .001);
  ActuatorBounds result{(-magnitude).cwiseMax((previous.array() - delta).matrix()),
                        magnitude.cwiseMin((previous.array() + delta).matrix())};
  if ((result.lower.array() > result.upper.array()).any())
    return std::nullopt;
  return result;
}
}  // namespace teleop_controllers
