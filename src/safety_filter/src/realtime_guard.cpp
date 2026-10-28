// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#include <chrono>
#include <cmath>

#include "safety_filter/cbf_safety_filter.hpp"

namespace safety_filter::cbf {
SafetyResult CbfSafetyFilter::guard(const Vector14d& preferred, const Vector14d& braking,
                                    const DualArmState& state, double dt, int own, double peer_age,
                                    double reserve, double peer_reserve) {
  const auto started = std::chrono::steady_clock::now();
  SafetyResult result;
  guard_recovery_ = true;
  guard_reserve_ = reserve;
  guard_peer_reserve_ = peer_reserve;
  for (int arm = 0; arm < 2; ++arm)
    actuator_anchor_.segment<7>(arm * 7) = state.arm[arm].previous_torque;
  buildConstraints(state, dt, &result);
  guard_recovery_ = false;
  guard_reserve_ = 0;
  guard_peer_reserve_ = 0;
  if (invalid_constraint_) {
    result.fatal_input_fault = true;
    result.solver_status = "invalid_local_constraint";
    return result;
  }
  // Eliminate the peer conservatively over its possible actuator interval.
  // Never assume it will apply a cached worker torque. Age widens the interval.
  const int peer = 1 - own;
  const double peer_slew = parameters_.maximum_torque_rate * std::clamp(peer_age + .001, .001, .02);
  for (int row : active_constraint_rows_) {
    double worst_min = 0, worst_max = 0;
    bool own_support = false;
    for (int j = 0; j < 7; ++j) {
      own_support |= std::abs(a_values_[(own * 7 + j) * constraint_count_ + row]) > 0;
      const int column = peer * 7 + j;
      const double a = a_values_[column * constraint_count_ + row];
      const double cap = parameters_.maximum_joint_torque[j];
      const double lo = std::max(-cap, state.arm[peer].previous_torque[j] - peer_slew);
      const double hi = std::min(cap, state.arm[peer].previous_torque[j] + peer_slew);
      worst_min += a >= 0 ? a * lo : a * hi;
      worst_max += a >= 0 ? a * hi : a * lo;
      a_values_[column * constraint_count_ + row] = 0;
    }
    int owner = -1;
    if (row < joint_position_row_begin_)
      owner = row / 7;
    else if (row < joint_velocity_row_begin_)
      owner = (row - joint_position_row_begin_) / 14;
    else if (row < joint_acceleration_row_begin_)
      owner = (row - joint_velocity_row_begin_) / 14;
    else if (row < singularity_row_begin_)
      owner = (row - joint_acceleration_row_begin_) / 7;
    else if (row < table_row_begin_)
      owner = row - singularity_row_begin_;
    else if (row < self_row_begin_)
      owner = (row - table_row_begin_) / (2 * static_cast<int>(capsules_.size()));
    else if (row < inter_arm_row_begin_)
      owner = (row - self_row_begin_) / static_cast<int>(self_pairs_.size());
    else if (row >= obstacle_row_begin_ && !obstacles_.empty())
      owner = (row - obstacle_row_begin_) / static_cast<int>(capsules_.size() * obstacles_.size());
    // A zero-authority constraint of this arm is still a constraint. In
    // particular, never erase an unsatisfiable mandatory table row.
    if (!own_support && owner == peer) {
      lower_bound_[row] = -OSQP_INFTY;
      upper_bound_[row] = OSQP_INFTY;
    } else {
      if (lower_bound_[row] > -.5 * OSQP_INFTY)
        lower_bound_[row] -= worst_min;
      if (upper_bound_[row] < .5 * OSQP_INFTY)
        upper_bound_[row] -= worst_max;
    }
  }
  const auto hard = [&](int row) {
    return row < joint_position_row_begin_ || (row >= table_row_begin_ && row < self_row_begin_);
  };
  const auto violation = [&](const Vector14d& value, bool mandatory, bool normalized) {
    double maximum = 0;
    if (!value.allFinite())
      return double(OSQP_INFTY);
    for (int row : active_constraint_rows_) {
      if (mandatory && !hard(row))
        continue;
      double dot = 0, squared = 0;
      for (int j = own * 7; j < own * 7 + 7; ++j) {
        const double a = a_values_[j * constraint_count_ + row];
        dot += a * value[j];
        squared += a * a;
      }
      const double residual =
          std::max(double(lower_bound_[row]) - dot, dot - double(upper_bound_[row]));
      maximum = std::max(maximum, residual / (normalized ? std::max(1., std::sqrt(squared)) : 1.));
    }
    return maximum;
  };
  const double tolerance = parameters_.solver_feasibility_tolerance;
  const auto bounded = [&](const Vector14d& value) { return clampToActuatorBounds(value, dt); };
  const auto project = [&](Vector14d& candidate, bool mandatory_only) {
    for (int row : active_constraint_rows_) {
      if (mandatory_only && !hard(row))
        continue;
      double dot = 0, squared = 0;
      for (int j = own * 7; j < own * 7 + 7; ++j) {
        const double a = a_values_[j * constraint_count_ + row];
        dot += a * candidate[j];
        squared += a * a;
      }
      if (squared < 1e-20)
        continue;
      const double correction = dot < lower_bound_[row]   ? lower_bound_[row] - dot
                                : dot > upper_bound_[row] ? upper_bound_[row] - dot
                                                          : 0;
      if (correction != 0) {
        for (int j = own * 7; j < own * 7 + 7; ++j)
          candidate[j] += correction * a_values_[j * constraint_count_ + row] / squared;
        candidate = bounded(candidate);
      }
    }
  };
  Vector14d chosen = bounded(preferred);
  bool complete = violation(chosen, false, false) <= tolerance;
  int repair_passes = 0;
  // Changing references need not turn ordinary, feasible safety intervention
  // into a latched stop-and-hold. First correct the current tracking candidate;
  // accept it only after checking every active row at the current state.
  while (!complete && repair_passes < 4) {
    project(chosen, false);
    ++repair_passes;
    complete = violation(chosen, false, false) <= tolerance;
  }
  bool tracking_repaired = complete && repair_passes > 0;
  bool tracking_limited = false;
  if (!complete) {
    // A short projection budget can miss a feasible point in coupled dynamics.
    // Use a currently verified seed, then retain as much of the repaired
    // tracking direction as every affine constraint permits. This is a bounded
    // line calculation, not a solver or an unchecked replay of old torque.
    Vector14d seed = bounded(actuator_anchor_);
    if (violation(seed, false, false) > tolerance)
      seed = bounded(braking);
    if (violation(seed, false, false) > tolerance) {
      // Zero modeled acceleration is another useful seed near rest. It is
      // accepted only after actuator clamping and all current geometry/joint
      // checks; compensation alone is never assumed safe near a boundary.
      Vector14d compensation = Vector14d::Zero();
      compensation.segment<7>(own * 7) = state.arm[own].coriolis;
      seed = bounded(compensation);
    }
    if (violation(seed, false, false) <= tolerance) {
      const Vector14d direction = chosen - seed;
      double fraction = 1.;
      for (int row : active_constraint_rows_) {
        double origin = 0, slope = 0;
        for (int j = own * 7; j < own * 7 + 7; ++j) {
          const double a = a_values_[j * constraint_count_ + row];
          origin += a * seed[j];
          slope += a * direction[j];
        }
        if (slope > 0 && upper_bound_[row] < .5 * OSQP_INFTY)
          fraction = std::min(fraction, (upper_bound_[row] + tolerance - origin) / slope);
        if (slope < 0 && lower_bound_[row] > -.5 * OSQP_INFTY)
          fraction = std::min(fraction, (lower_bound_[row] - tolerance - origin) / slope);
      }
      // Stay inside the computed interval in the presence of roundoff. The
      // same original-unit tolerance and independent full check still apply.
      const Vector14d limited = seed + std::clamp(fraction * (1. - 1e-9), 0., 1.) * direction;
      if (violation(limited, false, false) <= tolerance) {
        chosen = limited;
        complete = tracking_limited = true;
      }
    }
  }
  if (!complete) {
    // Finite constraint-repair sweeps for a braking/holding command. No solver,
    // convergence wait, allocation, or autonomous target retreat in this path.
    Vector14d candidate = bounded(braking);
    Vector14d best = candidate;
    double best_score = OSQP_INFTY;
    bool found_hard = violation(candidate, true, false) <= tolerance;
    if (found_hard)
      best_score = violation(candidate, false, true);
    // A failed repair is not evidence of infeasibility. Also consider the
    // actual current robot torque, bounded and rechecked against current rows,
    // before escalating. This is never an unchecked previous-worker replay.
    const Vector14d anchor = bounded(actuator_anchor_);
    const double anchor_score = violation(anchor, false, true);
    if (violation(anchor, true, false) <= tolerance && (!found_hard || anchor_score < best_score)) {
      found_hard = true;
      best = anchor;
      best_score = anchor_score;
    }
    // Share the original twelve-sweep budget with tracking correction. The
    // final four prioritize mandatory rows if no mandatory-valid candidate
    // exists. Otherwise retain that candidate while continuing full repair;
    // dropping secondary rows at sweep eight can strand a nearly feasible
    // torque just outside tolerance and cause a false hold.
    for (; repair_passes < 12; ++repair_passes) {
      project(candidate, repair_passes >= 8 && !found_hard);
      const double all = violation(candidate, false, false);
      const bool feasible_hard = violation(candidate, true, false) <= tolerance;
      const double score = violation(candidate, false, true);
      if (feasible_hard && (!found_hard || score < best_score)) {
        found_hard = true;
        best = candidate;
        best_score = score;
      }
      if (all <= tolerance) {
        best = candidate;
        found_hard = true;
        complete = true;
        break;
      }
    }
    chosen = best;
    result.fatal_solver_fault = !found_hard;
    // A fully verified command is ordinary constraint intervention, whichever
    // seed produced it. Do not latch a fault just because the first four
    // tracking sweeps missed it. Worker/model faults are latched separately by
    // AsyncSafetyController; genuinely relaxed constraints remain fallback.
    result.fallback_active = !complete;
    tracking_repaired |= complete;
  }
  result.safe_torque = chosen;
  result.solver_succeeded = complete;
  result.maximum_constraint_violation = violation(chosen, false, false);
  if (result.fatal_solver_fault) {
    for (int row : active_constraint_rows_) {
      if (!hard(row))
        continue;
      double dot = 0;
      for (int j = own * 7; j < own * 7 + 7; ++j)
        dot += a_values_[j * constraint_count_ + row] * chosen[j];
      const double residual =
          std::max(double(lower_bound_[row]) - dot, dot - double(upper_bound_[row]));
      if (residual > result.maximum_mandatory_constraint_violation) {
        result.maximum_mandatory_constraint_violation = residual;
        result.worst_mandatory_row = row;
      }
    }
  }
  result.intervention_norm = (chosen.segment<7>(own * 7) - preferred.segment<7>(own * 7)).norm();
  result.arm_intervention_norm[own] = result.intervention_norm;
  result.solver_status =
      result.fatal_solver_fault ? "no_valid_mandatory_table_command"
      : complete                ? (result.fallback_active ? "local_braking_constraints_satisfied"
                                                          : (tracking_limited    ? "local_candidate_limited"
                                                             : tracking_repaired ? "local_candidate_repaired"
                                                                                 : "local_candidate_verified"))
                                : "local_braking_secondary_constraints_relaxed";
  result.filter_time_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  return result;
}
}  // namespace safety_filter::cbf
