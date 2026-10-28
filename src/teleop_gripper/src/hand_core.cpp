#include "teleop_gripper/hand_core.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_gripper {
bool valid_hand_feedback(double width, double maximum) noexcept {
  return std::isfinite(width) && std::isfinite(maximum) && maximum > 0 &&
         width >= -kWidthReadbackTolerance && width <= maximum + kWidthReadbackTolerance;
}
double inward_joystick(double horizontal, bool right_controller) noexcept {
  return right_controller ? -horizontal : horizontal;
}
bool HandParameters::valid() const noexcept {
  return std::isfinite(engage) && std::isfinite(neutral) && neutral >= 0 && engage > neutral &&
         engage <= 1 && std::isfinite(speed) && speed > 0 && std::isfinite(open_speed) &&
         open_speed > 0 && std::isfinite(open_width) && (open_width == -1 || open_width > 0) &&
         std::isfinite(close_width) && close_width >= 0 &&
         (open_width == -1 || close_width < open_width) && std::isfinite(cable_diameter) &&
         cable_diameter > 0 && std::isfinite(cable_tolerance) && cable_tolerance >= 0 &&
         cable_diameter - cable_tolerance > close_width && std::isfinite(grasp_force) &&
         (grasp_force == -1 || (grasp_force > 0 && grasp_force <= 70)) && timeout_ns > 0 &&
         timeout_ns <= 100000000 && uncertainty_ns > 0 && uncertainty_ns <= 10000000;
}
void HandCore::reset_input() noexcept {
  need_neutral_ = true;
  direction_ = consumed_direction_ = 0;
}
bool HandCore::fresh(uint64_t now) const noexcept {
  return last_.source_ns > 0 &&
         (last_.source_ns <= now ? now - last_.source_ns <= p_.timeout_ns
                                 : last_.source_ns - now <= 10000000) &&
         last_.uncertainty_ns <= p_.uncertainty_ns;
}
HandRequest HandCore::interrupt() noexcept {
  if (!busy() || stopping_)
    return {};
  stopping_ = true;
  message_ = "stop unfinished finger motion; preserve an already confirmed grasp";
  return {HandOperation::stop};
}
HandRequest HandCore::start(HandOperation operation) noexcept {
  if (!calibrated_ || busy() || shutdown_)
    return {};
  const double open = p_.open_width == -1 ? maximum_width_ : p_.open_width;
  if (open > maximum_width_ || p_.close_width >= open) {
    message_ = "configured opening exceeds calibrated travel or closing limit";
    return {};
  }
  HandRequest request{operation, operation == HandOperation::close ? p_.close_width : open,
                      operation == HandOperation::close ? p_.speed : p_.open_speed};
  if (operation == HandOperation::close) {
    if (p_.grasp_force <= 0) {
      message_ = "closing disabled: configure grasp_force_n for the cable before use";
      return {};
    }
    if (p_.cable_diameter + p_.cable_tolerance > open) {
      message_ = "configured cable diameter/tolerance exceeds opening travel";
      return {};
    }
    request.force = p_.grasp_force;
    request.minimum_grasp_width = p_.cable_diameter - p_.cable_tolerance;
    request.maximum_grasp_width = p_.cable_diameter + p_.cable_tolerance;
    // The SDK's zero-target grasp window must include the actual cable gap.
    // Readback additionally enforces the lower cable bound: empty closure is
    // never accepted as a cable grasp even when firmware reports success.
    request.epsilon_inner = kWidthReadbackTolerance;
    request.epsilon_outer = request.maximum_grasp_width - request.width + kWidthReadbackTolerance;
  }
  operation_ = operation;
  if (operation == HandOperation::open)
    state_ = HandState::unknown;  // Explicit opening relinquishes any force hold.
  message_ = operation == HandOperation::close ? "closing at fixed speed while joystick inward"
                                               : "opening at fixed speed while joystick outward";
  return request;
}
HandRequest HandCore::input(const GripInput& c, uint64_t now) noexcept {
  if (shutdown_)
    return {};
  if (operation_ == HandOperation::calibrate) {
    reset_input();
    return {};
  }
  const bool source_fresh =
      c.source_ns > 0 &&
      (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns : c.source_ns - now <= 10000000) &&
      c.uncertainty_ns <= p_.uncertainty_ns;
  if (!c.valid || !c.joystick_active || !source_fresh || !std::isfinite(c.value) ||
      std::abs(c.value) > 1 || c.session[0] == '\0') {
    reset_input();
    const auto stop = interrupt();
    message_ =
        "input unavailable/stale; motion stopped, confirmed grasp preserved; neutral required";
    return stop;
  }
  if (!identity_ || c.session != last_.session || c.generation != last_.generation) {
    reset_input();
    identity_ = true;
    last_ = GripInput{};
    if (busy()) {
      last_ = c;
      return interrupt();
    }
  }
  if (last_.source_ns && (c.sequence <= last_.sequence || c.source_ns <= last_.source_ns ||
                          c.source_ns - last_.source_ns > p_.timeout_ns)) {
    reset_input();
    const auto stop = interrupt();
    message_ = "joystick order/gap fault; confirmed grasp preserved; neutral required";
    if (c.sequence > last_.sequence && c.source_ns > last_.source_ns)
      last_ = c;
    return stop;
  }
  last_ = c;
  if (std::abs(c.value) <= p_.neutral) {
    direction_ = consumed_direction_ = 0;
    need_neutral_ = false;
    return interrupt();
  }
  if (need_neutral_)
    return {};
  int direction = 0;
  if (c.value >= p_.engage || (direction_ > 0 && c.value > p_.neutral))
    direction = 1;
  else if (c.value <= -p_.engage || (direction_ < 0 && c.value < -p_.neutral))
    direction = -1;
  if (direction != direction_)
    consumed_direction_ = 0;
  direction_ = direction;
  if (!direction)
    return interrupt();
  const auto desired = direction > 0 ? HandOperation::close : HandOperation::open;
  if (busy())
    return stopping_ || operation_ == desired ? HandRequest{} : interrupt();
  if (consumed_direction_ == direction || (direction > 0 && grasp_confirmed()))
    return {};
  consumed_direction_ = direction;
  return start(desired);
}
HandRequest HandCore::calibrate(bool empty_hand, bool confirmed) noexcept {
  if (!empty_hand || !confirmed || busy() || shutdown_) {
    message_ = "calibration requires both confirmations and an idle worker";
    return {};
  }
  reset_input();
  calibrated_ = false;
  state_ = HandState::unknown;
  operation_ = HandOperation::calibrate;
  message_ = "calibration started";
  return {operation_};
}
HandRequest HandCore::complete(HandOperation operation, bool success, bool communication_ok,
                               double width, double maximum, uint64_t now, bool grasped) noexcept {
  const bool stopped = stopping_;
  operation_ = HandOperation::none;
  stopping_ = false;
  if (!communication_ok || !valid_hand_feedback(width, maximum)) {
    calibrated_ = false;
    state_ = HandState::unknown;
    reset_input();
    message_ = "Hand communication/state fault; explicit empty-hand calibration required";
    return {};
  }
  width_ = std::clamp(width, 0., maximum);
  maximum_width_ = maximum;
  if (operation == HandOperation::calibrate) {
    calibrated_ = success && !stopped;
    state_ = HandState::unknown;
    reset_input();
    message_ = calibrated_ ? "calibrated; joystick neutral required before movement"
                           : "calibration failed or stopped";
    return {};
  }
  const bool cable_grasp = operation == HandOperation::close && success && grasped &&
                           width_ >= p_.cable_diameter - p_.cable_tolerance &&
                           width_ <= p_.cable_diameter + p_.cable_tolerance;
  if (cable_grasp) {
    state_ = HandState::holding;
    message_ = "cable grasp confirmed; force hold persists through neutral and Quest input loss";
  } else if (!success && !stopped) {
    state_ = HandState::unknown;
    reset_input();
    message_ = "grasp/move failed; no retry or force increase; neutral required";
    return {};
  } else {
    state_ = stopped ? HandState::stopped
                     : (operation == HandOperation::open ? HandState::open : HandState::closed);
    message_ = stopped
                   ? "finger motion stopped"
                   : (operation == HandOperation::open ? "opening completed"
                                                       : "closed without a confirmed cable grasp");
  }
  // Only the latest still-held direction may run after an acknowledged stop.
  // Neutral, input faults, shutdown and expired samples cannot replay a goal.
  if (stopped && !shutdown_ && !need_neutral_ && direction_ && fresh(now)) {
    if (direction_ > 0 && cable_grasp)
      return {};
    consumed_direction_ = direction_;
    return start(direction_ > 0 ? HandOperation::close : HandOperation::open);
  }
  return {};
}
HandRequest HandCore::watchdog(uint64_t now) noexcept {
  if (operation_ != HandOperation::calibrate && last_.source_ns && !fresh(now)) {
    reset_input();
    const auto stop = interrupt();
    message_ = "Quest input timeout; motion stopped, confirmed grasp preserved; neutral required";
    return stop;
  }
  return {};
}
HandRequest HandCore::shutdown() noexcept {
  shutdown_ = true;
  reset_input();
  return interrupt();
}
}  // namespace teleop_gripper
