#include "teleop_controllers/controller_manager.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_controllers {
const char* reason_name(Reason reason) noexcept {
  switch (reason) {
    case Reason::tracking_limit:
      return "protective_tracking_limit";
    case Reason::input_velocity_limit:
      return "protective_input_velocity_limit";
    case Reason::startup:
      return "startup";
    case Reason::released:
      return "released";
    case Reason::active:
      return "active";
    case Reason::orientation_only:
      return "orientation_only";
    case Reason::invalid_command:
      return "invalid_command";
    case Reason::stale:
      return "stale";
    case Reason::order:
      return "duplicate_or_out_of_order";
    case Reason::session_change:
      return "session_or_reference_change";
    case Reason::envelope:
      return "outer_envelope";
    case Reason::queue_overflow:
      return "command_queue_overflow";
    case Reason::invalid_state:
      return "invalid_robot_state";
    case Reason::shutdown:
      return "shutdown";
    case Reason::homing:
      return "homing_owns_nominal_control";
    case Reason::position_hold:
      return "position_hold";
  }
  return "unknown";
}

const char* state_fault_name(StateFault fault) noexcept {
  switch (fault) {
    case StateFault::none:
      return "none";
    case StateFault::nonfinite_state:
      return "nonfinite_robot_state";
    case StateFault::orientation:
      return "invalid_robot_orientation";
    case StateFault::period:
      return "invalid_callback_period";
    case StateFault::nonfinite_nominal:
      return "nonfinite_nominal_torque";
    case StateFault::actuator_interval:
      return "empty_actuator_interval";
    case StateFault::invalid_reference:
      return "invalid_controller_reference";
  }
  return "unknown";
}

bool ManagerParameters::valid() const noexcept {
  const double numbers[] = {chain_radius,
                            torque_rate,
                            reference_linear_speed,
                            reference_angular_speed,
                            reference_linear_acceleration,
                            reference_angular_acceleration,
                            tracking_position_limit,
                            tracking_orientation_limit};
  for (double value : numbers)
    if (!std::isfinite(value) || value <= 0)
      return false;
  if (!envelope_center.allFinite() || !std::isfinite(tool_offset) || tool_offset < 0 ||
      !std::isfinite(envelope_margin) || envelope_margin < 0 || torque_rate > 1000 ||
      timeout_ns == 0 || uncertainty_ns == 0)
    return false;
  const Vector7 ceilings = (Vector7() << 87, 87, 87, 87, 12, 12, 12).finished();
  return torque_limits.allFinite() && (torque_limits.array() > 0).all() &&
         (torque_limits.array() <= ceilings.array()).all();
}

bool ControllerManager::fresh(const Command& c, uint64_t now) const noexcept {
  return c.source_ns > 0 && c.receipt_ns > 0 && now >= c.receipt_ns &&
         now - c.receipt_ns <= p_.timeout_ns && c.uncertainty_ns <= p_.uncertainty_ns &&
         (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns
                             : c.source_ns - now <= p_.future_ns);
}

void ControllerManager::fault(Reason reason) noexcept {
  if (protective_stop_) {
    enabled_ = false;
    reason_ = protective_reason_;
    return;
  }
  teleoperation_reset_pending_ = true;
  const bool exit_hold =
      reason == Reason::shutdown || reason == Reason::invalid_state || reason == Reason::homing;
  if (exit_hold) {
    local_hold_ = false;
    hold_reset_pending_ = true;
  } else if (enabled_) {
    // Stop pursuing remote targets and capture measured joints once. Subsequent
    // input faults must preserve that local anchor and the stiffness ramp.
    local_hold_ = true;
    hold_reset_pending_ = true;
  }
  // Homing arbitration must not turn a preceding input fault into permission
  // to hold. Only a fresh, explicit release restores that permission.
  if (reason != Reason::homing)
    hold_permitted_ = false;
  enabled_ = false;
  release_seen_ = false;
  reason_ = reason;
}

void ControllerManager::accept(const Command& c, uint64_t now) noexcept {
  if (shutdown_ || protective_stop_)
    return;
  if (!c.valid || c.session[0] == '\0') {
    fault(Reason::invalid_command);
    return;
  }
  if (!fresh(c, now)) {
    fault(Reason::stale);
    return;
  }
  if (!identity_ || c.session != session_ || c.generation != generation_) {
    fault(Reason::session_change);
    session_ = c.session;
    generation_ = c.generation;
    identity_ = true;
    ordered_ = false;
  }
  if (ordered_ && (c.sequence <= sequence_ || c.source_ns <= source_ns_)) {
    // Mapper watchdog repeats a valid release. It cannot refresh freshness or
    // authorize a new release; enabled/reordered commands always fault.
    if (!c.enabled && !last_.enabled && c.released == last_.released && c.sequence == sequence_ &&
        c.source_ns == source_ns_)
      return;
    fault(Reason::order);
    return;
  }
  ordered_ = true;
  sequence_ = c.sequence;
  source_ns_ = c.source_ns;
  last_ = c;
  if (!c.enabled) {
    if (enabled_) {
      teleoperation_reset_pending_ = true;
      local_hold_ = true;
      hold_reset_pending_ = true;
    }
    enabled_ = false;
    if (c.released) {
      last_release_source_ns_ = c.source_ns;
      release_seen_ = true;
      if (!local_hold_)
        hold_reset_pending_ = true;
      local_hold_ = true;
      hold_permitted_ = true;
      reason_ = Reason::released;
    } else if (release_seen_)
      reason_ = Reason::released;
    return;
  }
  if (!c.position.allFinite() || !c.orientation.coeffs().allFinite() ||
      std::abs(c.orientation.norm() - 1) > 0.01 || !c.twist.allFinite() || c.released) {
    fault(Reason::invalid_command);
    return;
  }
  // Finite but implausible feedforward must not produce a torque impulse.
  if (c.twist.head<3>().norm() > 1.0 || c.twist.tail<3>().norm() > 6.0) {
    protect(Reason::input_velocity_limit);
    return;
  }
  if ((c.position - p_.envelope_center).norm() >
      p_.chain_radius + p_.tool_offset + p_.envelope_margin) {
    fault(Reason::envelope);
    return;
  }
  if (!release_seen_) {
    enabled_ = false;
    return;
  }
  enabled_ = true;
  local_hold_ = false;
  hold_permitted_ = false;
  hold_reset_pending_ = true;
  reason_ = Reason::active;
}

void ControllerManager::shutdown() noexcept {
  shutdown_ = true;
  fault(Reason::shutdown);
  homing_.cancel("shutdown");
}

namespace {
Eigen::Vector3d bounded(Eigen::Vector3d v, double limit) noexcept {
  const double norm = v.norm();
  if (norm > limit)
    v *= limit / norm;
  return v;
}
Eigen::Vector3d rotation_vector(Eigen::Quaterniond q) noexcept {
  q.normalize();
  if (q.w() < 0)
    q.coeffs() *= -1;
  const Eigen::AngleAxisd a(q);
  return a.axis() * a.angle();
}
}  // namespace

void ControllerManager::protect(Reason reason) noexcept {
  if (protective_stop_)
    return;
  fault(reason);
  protective_stop_ = true;
  protective_reason_ = reason;
  reason_ = reason;
  local_hold_ = false;
  homing_.cancel("latched motion protection");
}

Vector7 ControllerManager::protective_torque(const ArmState& s) noexcept {
  // Do not engage the high-stiffness parked law while the arm is moving.
  // Capture once after braking. Bound acceleration demand before mapping to
  // torque, including the hold correction; runtime actuator clipping follows.
  if (!protective_holding_ && s.dq.cwiseAbs().maxCoeff() < .02) {
    protective_q_ = s.q;
    protective_holding_ = true;
  }
  Vector7 a = -20.0 * s.dq;
  if (protective_holding_)
    a = 25.0 * (protective_q_ - s.q) - 10.0 * s.dq;
  a = a.cwiseMax(-2.0).cwiseMin(2.0);
  return s.coriolis + s.mass * a;
}

CartesianReference ControllerManager::govern(const ArmState& s, double dt, bool reset) noexcept {
  if (reset) {
    governed_ = CartesianReference{s.position, s.orientation, Vector6::Zero()};
    blocked_seconds_ = 0;
  }
  const Eigen::Vector3d goal = last_.orientation_only ? orientation_hold_position_ : last_.position;
  const Eigen::Vector3d displacement = goal - governed_.position;
  const Eigen::Vector3d rotation =
      rotation_vector(last_.orientation * governed_.orientation.conjugate());
  // Native/pose-derived feedforward biases the trajectory velocity, but both
  // pose and velocity sent to impedance come from this single bounded trajectory.
  Eigen::Vector3d v = last_.twist.head<3>() + 4.0 * displacement;
  Eigen::Vector3d w = last_.twist.tail<3>() + 4.0 * rotation;
  v = bounded(v, std::min(p_.reference_linear_speed,
                          std::sqrt(2 * p_.reference_linear_acceleration * displacement.norm())));
  w = bounded(w, std::min(p_.reference_angular_speed,
                          std::sqrt(2 * p_.reference_angular_acceleration * rotation.norm())));
  if (last_.orientation_only)
    v.setZero();
  const Eigen::Vector3d old_v = governed_.twist.head<3>(), old_w = governed_.twist.tail<3>();
  v = old_v + bounded(v - old_v, p_.reference_linear_acceleration * dt);
  w = old_w + bounded(w - old_w, p_.reference_angular_acceleration * dt);
  governed_.twist << v, w;
  governed_.position += v * dt;
  const double angle = w.norm() * dt;
  if (angle > 1e-12)
    governed_.orientation =
        (Eigen::Quaterniond(Eigen::AngleAxisd(angle, w.normalized())) * governed_.orientation)
            .normalized();
  return governed_;
}

ManagerOutput ControllerManager::update(const ArmState& s, uint64_t now, double dt) noexcept {
  ManagerOutput result;
  auto& out = result.control;
  if (identity_ && !fresh(last_, now))
    fault(Reason::stale);
  if (!s.position.allFinite() || !s.orientation.coeffs().allFinite() || !s.q.allFinite() ||
      !s.dq.allFinite() || !s.coriolis.allFinite() || !s.jacobian.allFinite() ||
      !s.previous_torque.allFinite() || !s.mass.allFinite())
    out.state_fault = StateFault::nonfinite_state;
  else if (std::abs(s.orientation.norm() - 1) > 0.01)
    out.state_fault = StateFault::orientation;
  else if (!std::isfinite(dt) || dt <= 0 || dt > 0.02)
    out.state_fault = StateFault::period;
  if (out.state_fault == StateFault::none) {
    const double slew = p_.torque_rate * std::min(dt, 0.001);
    for (int j = 0; j < 7; ++j)
      if (std::max(-p_.torque_limits[j], s.previous_torque[j] - slew) >
          std::min(p_.torque_limits[j], s.previous_torque[j] + slew))
        out.state_fault = StateFault::actuator_interval;
  }
  Eigen::LDLT<Eigen::Matrix<double, 7, 7>> model_factor;
  if (out.state_fault == StateFault::none) {
    model_factor.compute(s.mass);
    if (!s.mass.isApprox(s.mass.transpose(), 1e-9) || model_factor.info() != Eigen::Success ||
        !model_factor.isPositive())
      out.state_fault = StateFault::nonfinite_state;
  }
  if (out.state_fault != StateFault::none) {
    fault(Reason::invalid_state);
    out.reason = Reason::invalid_state;
    return result;  // Runtime must terminate; never send placeholder torques.
  }

  const bool reset_reference =
      teleoperation_reset_pending_ ||
      (enabled_ && selected_ != (last_.orientation_only ? &orientation_only_ : &teleoperation_));
  if (enabled_ && p_.tracking_protection_enabled && p_.reference_shaping_enabled &&
      !reset_reference &&
      ((s.position - governed_.position).norm() > p_.tracking_position_limit ||
       rotation_vector(s.orientation * governed_.orientation.conjugate()).norm() >
           p_.tracking_orientation_limit))
    protect(Reason::tracking_limit);

  if (protective_stop_) {
    out.reason = protective_reason_;
    out.protective_stop = true;
    out.target_position = s.position;
    out.target_orientation = s.orientation;
    out.nominal = protective_torque(s);
    if (!out.nominal.allFinite()) {
      out.state_fault = StateFault::nonfinite_nominal;
      out.reason = Reason::invalid_state;
    }
    return result;
  }

  const bool was_homing = homing_.active();
  result.homing = homing_.update(s, now, dt);
  if (was_homing || result.homing.active)
    fault(Reason::homing);
  if (shutdown_)
    fault(Reason::shutdown);

  out.target_position = enabled_ ? last_.position : s.position;
  out.target_orientation = enabled_ ? last_.orientation : s.orientation;
  out.desired_twist = enabled_ ? last_.twist : Vector6::Zero();
  out.motion_enabled = enabled_;
  out.reason = enabled_ && last_.orientation_only ? Reason::orientation_only : reason_;
  out.source_ns = last_.source_ns;
  out.sequence = last_.sequence;

  if (result.homing.stop_failed)
    return result;  // Fatal controlled-stop failure.
  TorqueController* law =
      enabled_ ? (last_.orientation_only ? &orientation_only_ : &teleoperation_) : &damping_;
  if (enabled_ && last_.orientation_only) {
    if (selected_ != &orientation_only_)
      orientation_hold_position_ = s.position;
    out.target_position = orientation_hold_position_;
    out.desired_twist.head<3>().setZero();
  }
  if (enabled_) {
    const auto trajectory =
        p_.reference_shaping_enabled
            ? govern(s, dt, reset_reference)
            : CartesianReference{out.target_position, out.target_orientation, out.desired_twist};
    out.target_position = trajectory.position;
    out.target_orientation = trajectory.orientation;
    out.desired_twist = trajectory.twist;
    const double position_error = (s.position - trajectory.position).norm();
    const double angle_error =
        rotation_vector(s.orientation * trajectory.orientation.conjugate()).norm();
    // Persistent lack of tracking cannot wind up the servo reference, even
    // when the wrists remain stuck and the proximal arm can compensate.
    if (p_.tracking_protection_enabled && (position_error > .5 * p_.tracking_position_limit ||
                                           angle_error > .5 * p_.tracking_orientation_limit))
      blocked_seconds_ += dt;
    else
      blocked_seconds_ = 0;
    if (p_.tracking_protection_enabled &&
        (position_error > p_.tracking_position_limit ||
         angle_error > p_.tracking_orientation_limit || blocked_seconds_ > .25)) {
      protect(Reason::tracking_limit);
      out.reason = protective_reason_;
      out.protective_stop = true;
      out.motion_enabled = false;
      out.desired_twist.setZero();
      out.nominal = protective_torque(s);
      if (!out.nominal.allFinite()) {
        out.state_fault = StateFault::nonfinite_nominal;
        out.reason = Reason::invalid_state;
      }
      return result;
    }
  }
  ControlReference reference =
      CartesianReference{out.target_position, out.target_orientation, out.desired_twist};
  if (result.homing.active) {
    law = &homing_;
    reference = JointReference{result.homing.desired_q, result.homing.desired_dq};
  } else if (!enabled_ && !shutdown_ &&
             (local_hold_ ||
              (hold_permitted_ && (reason_ == Reason::startup || reason_ == Reason::released ||
                                   (reason_ == Reason::homing && last_.valid && last_.released &&
                                    !last_.enabled && fresh(last_, now)))))) {
    // A deliberate release holds the measured arm configuration, never an old
    // Quest goal. Startup also holds locally without authorizing teleoperation.
    // Quest input loss does not release a parked arm. Invalid state, active homing and shutdown
    // retain their separate stop policy.
    law = &position_hold_;
    local_hold_ = true;
    if (selected_ != law || hold_reset_pending_) {
      hold_reference_ = JointReference{s.q, Vector7::Zero()};
      hold_position_ = s.position;
      hold_orientation_ = s.orientation;
    }
    reference = hold_reference_;
    out.target_position = hold_position_;
    out.target_orientation = hold_orientation_;
    out.position_hold_active = true;
    out.reason = Reason::position_hold;
  }
  if (law != selected_ ||
      ((law == &teleoperation_ || law == &orientation_only_) && teleoperation_reset_pending_) ||
      (law == &position_hold_ && hold_reset_pending_)) {
    law->reset(s);
    selected_ = law;
    if (law == &teleoperation_ || law == &orientation_only_)
      teleoperation_reset_pending_ = false;
    if (law == &position_hold_)
      hold_reset_pending_ = false;
  }
  const auto nominal = law->compute(s, reference, dt);
  out.nominal = nominal.torque;
  out.state_fault = nominal.fault;
  if (out.state_fault == StateFault::none && !out.nominal.allFinite())
    out.state_fault = StateFault::nonfinite_nominal;
  if (out.state_fault != StateFault::none) {
    fault(Reason::invalid_state);
    out.reason = reason_;
    out.motion_enabled = false;
    out.position_hold_active = false;
  }
  // The local runtime applies the same actuator clipping to every nominal law.
  return result;
}
}  // namespace teleop_controllers
