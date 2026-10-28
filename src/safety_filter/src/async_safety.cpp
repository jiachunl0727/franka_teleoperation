// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#include "safety_filter/async_safety.hpp"

#include <algorithm>
#include <cmath>
namespace safety_filter {
namespace {
bool valid_model(const cbf::ArmState& s) noexcept {
  if (!s.q.allFinite() || !s.dq.allFinite() || !s.coriolis.allFinite() ||
      !s.previous_torque.allFinite() || !s.mass.allFinite() ||
      !s.mass.isApprox(s.mass.transpose(), 1e-9))
    return false;
  const Eigen::LLT<cbf::Matrix7d> factor(s.mass);
  if (factor.info() != Eigen::Success)
    return false;
  for (int i = 0; i < cbf::kModelFrames; ++i) {
    const auto& pose = s.frame_pose[i];
    if (!pose.matrix().allFinite() || !s.frame_jacobian[i].allFinite() ||
        !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8) ||
        !(pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
        std::abs(pose.linear().determinant() - 1) > 1e-8)
      return false;
  }
  return true;
}
}  // namespace
bool AsyncSafetyController::configure(int arm, const CoordinatorParameters& limits,
                                      const cbf::SafetyParameters& safety,
                                      const std::array<cbf::ArmIdentity, 2>& ids,
                                      std::string* error) {
  if (arm < 0 || arm > 1 || !limits.valid() || !safety.active_arms[arm]) {
    if (error)
      *error = "invalid local safety controller configuration";
    return false;
  }
  arm_ = arm;
  limits_ = limits;
  safety_ = safety;
  auto local = safety;
  local.enabled = true;
  local.table_enabled = true;  // Mandatory irrespective of the overall CBF switch.
  if (!safety.enabled) {
    local.joint_position_enabled = local.joint_velocity_enabled = local.joint_acceleration_enabled =
        false;
    local.singularity_enabled = local.self_collision_enabled = local.inter_arm_collision_enabled =
        false;
    local.obstacle_collision_enabled = false;
  }
  if (!guard_.configure(local, ids, error, false))
    return false;
  local.active_arms[1 - arm] = false;
  return solo_guard_.configure(local, ids, error, false);
}
bool AsyncSafetyController::prepare_model(cbf::ArmState& s, uint64_t now) noexcept {
  model_reused_ = false;
  if (valid_model(s)) {
    last_model_ = s;
    model_ns_ = now;
    have_model_ = true;
    return true;
  }
  // Only model-data glitches receive grace. Joint feedback and torque anchors
  // must remain finite. Never extend the original good model's timestamp.
  if (!have_model_ || now < model_ns_ || now - model_ns_ > limits_.model_grace_ns ||
      !s.q.allFinite() || !s.dq.allFinite() || !s.previous_torque.allFinite() ||
      (s.q - last_model_.q).cwiseAbs().maxCoeff() > .01 ||
      (s.dq - last_model_.dq).cwiseAbs().maxCoeff() > .1)
    return false;
  const auto q = s.q.eval(), dq = s.dq.eval(), previous = s.previous_torque.eval();
  s = last_model_;
  s.q = q;
  s.dq = dq;
  s.previous_torque = previous;
  model_reused_ = true;
  return true;
}
cbf::Vector7d AsyncSafetyController::braking(const cbf::ArmState& s, double dt,
                                             bool reset) noexcept {
  if (!have_brake_ || reset) {
    brake_q_ = s.q;
    brake_dq_ = s.dq;
    holding_ = false;
    have_brake_ = true;
  }
  cbf::Vector7d acceleration = cbf::Vector7d::Zero();
  if (!holding_) {
    const auto previous = brake_dq_.eval();
    for (int j = 0; j < 7; ++j)
      brake_dq_[j] -= std::clamp(brake_dq_[j], -limits_.braking_acceleration * dt,
                                 limits_.braking_acceleration * dt);
    brake_q_ += .5 * (previous + brake_dq_) * dt;
    acceleration = (brake_dq_ - previous) / dt;
    if (brake_dq_.cwiseAbs().maxCoeff() < 1e-6 && s.dq.cwiseAbs().maxCoeff() < .02) {
      holding_ = true;
      brake_q_ = s.q;
      acceleration.setZero();
    }
  }
  return s.coriolis + s.mass * acceleration + limits_.hold_stiffness * (brake_q_ - s.q) +
         limits_.hold_damping * (brake_dq_ - s.dq);
}
AsyncSafetyOutput AsyncSafetyController::update(const ModelRequest& request,
                                                const ModelRequest* peer, const SafetyReply* result,
                                                uint64_t now) {
  AsyncSafetyOutput out;
  out.model_reused = model_reused_;
  out.reply.cycles[arm_] = request.cycle;
  if (!valid_model(request.state) || !std::isfinite(request.period) || request.period <= 0 ||
      request.period > .02) {
    out.reply.result.fatal_input_fault = true;
    out.reply.result.solver_status = "invalid_local_model";
    return out;
  }
  if (result && (!have_result_ || result->cycles[arm_] > cached_.cycles[arm_])) {
    cached_ = *result;
    have_result_ = true;
  }
  const int other = 1 - arm_;
  const bool need_peer = safety_.enabled && safety_.active_arms[other];
  out.peer_usable = !need_peer || (peer && peer->stamp_ns <= now &&
                                   now - peer->stamp_ns <= limits_.maximum_peer_age_ns &&
                                   valid_model(peer->state));
  if (have_result_ && cached_.source_ns[arm_] <= now)
    out.worker_age_ns = now - cached_.source_ns[arm_];
  const bool worker_model_usable =
      !safety_.enabled ||
      (have_result_ && cached_.source_ns[arm_] > 0 && cached_.source_ns[arm_] <= now &&
       out.worker_age_ns <= limits_.maximum_result_age_ns &&
       cached_.completed_ns >= cached_.source_ns[arm_] && cached_.completed_ns <= now &&
       cached_.result.solver_succeeded && !cached_.result.fallback_active &&
       !cached_.result.fatal_input_fault && cached_.result.safe_torque.allFinite() &&
       cached_.nominal.allFinite() && cached_.source_q[arm_].allFinite() &&
       cached_.source_dq[arm_].allFinite() &&
       (request.state.q - cached_.source_q[arm_]).cwiseAbs().maxCoeff() <=
           limits_.result_joint_change &&
       (request.state.dq - cached_.source_dq[arm_]).cwiseAbs().maxCoeff() <=
           limits_.result_velocity_change);
  out.worker_usable =
      worker_model_usable && (!safety_.enabled || cached_.epoch[arm_] == request.epoch);
  const bool healthy = out.worker_usable && out.peer_usable && !model_reused_;
  if (pending_epoch_ != request.epoch) {
    pending_epoch_ = request.epoch;
    pending_since_ = now;
    have_brake_ = false;
    // A mode change alone is not an interruption of verified health. In
    // particular, a healthy released dwell must count toward re-engagement.
    // The health check below still resets the dwell for an unavailable or
    // rejected worker model certificate, peer or model.
  }
  // A fresh successful certificate of the unchanged physical state can keep
  // the health dwell running across a nominal-mode transition. Its torque is
  // still unusable until an exact current-epoch result arrives. Otherwise each
  // release/engagement throws away the entire released recovery dwell.
  if (worker_model_usable && out.peer_usable && !model_reused_) {
    if (!healthy_since_)
      healthy_since_ = now;
  } else
    healthy_since_ = 0;
  bool enter = false;
  const auto degrade = [&] {
    if (!degraded_) {
      degraded_ = true;
      enter = true;
      release_before_fault_ = request.release_source_ns;
      healthy_since_ = 0;  // Pre-fault health cannot satisfy post-fault recovery.
    }
  };
  const bool matching_failed = have_result_ && cached_.epoch[arm_] == request.epoch &&
                               (!cached_.result.solver_succeeded || cached_.result.fallback_active);
  if (request.motion_active && !healthy &&
      (matching_failed || model_reused_ || now - pending_since_ > limits_.maximum_result_age_ns))
    degrade();
  // A fresh release after the fault plus a new engagement is mandatory. A
  // healthy worker alone never re-enables an old held trigger or homing request.
  if (degraded_ && healthy && request.motion_active &&
      request.release_source_ns > release_before_fault_ && healthy_since_ &&
      now - healthy_since_ >= limits_.recovery_dwell_ns) {
    degraded_ = false;
    have_brake_ = false;
  }
  const auto backup =
      braking(request.state, request.period, enter || (!degraded_ && request.motion_active));
  cbf::DualArmState state;
  state.arm[arm_] = request.state;
  if (need_peer && out.peer_usable)
    state.arm[other] = peer->state;
  cbf::Vector14d preferred = cbf::Vector14d::Zero(), brake = preferred;
  brake.segment<7>(arm_ * 7) = backup;
  preferred = brake;
  if (!degraded_ && healthy) {
    preferred.segment<7>(arm_ * 7) = request.nominal;
    if (safety_.enabled)
      preferred.segment<7>(arm_ * 7) +=
          cached_.result.safe_torque.segment<7>(arm_ * 7) - cached_.nominal.segment<7>(arm_ * 7);
  }
  const double peer_age = need_peer && out.peer_usable ? (now - peer->stamp_ns) * 1e-9 : 0;
  // Bound link displacement during peer/model age. This is an explicit sampled
  // model approximation, not an unlimited stale-state certificate.
  const double reserve =
      limits_.clearance_reserve +
      (model_reused_ ? 1.5 * (request.state.q - last_model_.q).cwiseAbs().sum() : 0.0);
  auto& checker = need_peer && out.peer_usable ? guard_ : solo_guard_;
  if (out.peer_usable != peer_was_usable_)
    checker.reset();
  peer_was_usable_ = out.peer_usable;
  out.reply.result = checker.guard(preferred, brake, state, request.period, arm_, peer_age, reserve,
                                   limits_.peer_motion_bound * peer_age);
  out.reply.result.intervention_norm =
      (out.reply.result.safe_torque.segment<7>(arm_ * 7) - request.nominal).norm();
  out.reply.result.arm_intervention_norm[arm_] = out.reply.result.intervention_norm;

  if (request.motion_active && out.reply.result.fallback_active)
    degrade();
  out.entered_degraded = enter;
  out.degraded = degraded_;
  out.holding = holding_;
  out.reply.result.fallback_active |= degraded_ || !healthy;
  out.reply.result.solver_time_seconds = have_result_ ? cached_.result.solver_time_seconds : 0;
  out.reply.result.solver_iterations = have_result_ ? cached_.result.solver_iterations : 0;
  out.reply.source_ns = have_result_ ? cached_.source_ns : std::array<uint64_t, 2>{};
  out.reply.started_ns = now;
  out.reply.completed_ns = now;
  out.reply.consecutive_failures = have_result_ ? cached_.consecutive_failures : 0;
  if (have_result_) {
    out.worker_cycles = cached_.cycles;
    out.worker_status = cached_.result.solver_status;
  }
  out.reason = model_reused_                      ? "model_grace"
               : !out.peer_usable                 ? "peer_unavailable"
               : !out.worker_usable               ? "worker_unavailable_or_rejected"
               : degraded_                        ? "release_reengagement_required"
               : out.reply.result.fallback_active ? "local_constraint_intervention"
                                                  : "tracking";
  return out;
}
}  // namespace safety_filter
