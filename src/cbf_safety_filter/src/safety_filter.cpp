#include "cbf_safety_filter/safety_filter.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "cbf_safety_filter/recovery_projection.hpp"
namespace cbf_safety_filter {
namespace {
using Clock = std::chrono::steady_clock;
double us(Clock::time_point t) {
  return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}
// Solve the weighted box + ONE hard halfspace projection. If its minimizer
// satisfies every row, it is also the full QP optimum.
int most_violated_hard_row(const QpProblem& problem, const Vec7& candidate) noexcept {
  int selected = -1;
  double worst = 0;
  for (int i = 0; i < problem.active_rows; ++i) {
    const auto& row = problem.original[i];
    const double norm = row.g.norm();
    if (!std::isfinite(norm) || norm <= 1e-15)
      continue;
    const double violation = (row.g.dot(candidate) - row.bound) / norm;
    if (violation > worst) {
      worst = violation;
      selected = i;
    }
  }
  return selected;
}
bool project_one_hard_row(const QpProblem& problem, const Vec7& nominal, const TorqueBounds& box,
                          Vec7& candidate) noexcept {
  const int selected = most_violated_hard_row(problem, candidate);
  if (selected < 0)
    return false;
  const auto& row = problem.original[selected];
  const Vec7 direction = row.g.cwiseQuotient(problem.config.weights);
  if (!direction.allFinite())
    return false;
  // Beyond these breakpoints every coordinate minimizes g*x over the box.
  double high = 0;
  for (int j = 0; j < 7; ++j) {
    if (direction[j] != 0) {
      const double limit = row.g[j] > 0 ? box.lower[j] : box.upper[j];
      high = std::max(high, (nominal[j] - limit) / direction[j]);
    }
  }
  if (!std::isfinite(high) || high <= 0)
    return false;
  high = std::nextafter(high, std::numeric_limits<double>::infinity());
  if (!std::isfinite(high))
    return false;
  const auto at = [&](double multiplier) -> Vec7 {
    return (nominal - multiplier * direction).cwiseMax(box.lower).cwiseMin(box.upper);
  };
  const double high_residual = row.g.dot(at(high)) - row.bound;
  if (!at(high).allFinite() || !std::isfinite(high_residual) || high_residual > 0)
    return false;
  double low = 0;
  // Fixed work, no allocation. Keep the feasible side of the bracket, including
  // at floating-point saturation; uncertainty falls back to the native solver.
  for (int i = 0; i < 64; ++i) {
    const double mid = low + (high - low) * .5;
    if (mid == low || mid == high)
      break;
    const double residual = row.g.dot(at(mid)) - row.bound;
    if (!std::isfinite(residual))
      return false;
    if (residual > 0)
      low = mid;
    else
      high = mid;
  }
  candidate = at(high);
  if (!candidate.allFinite())
    return false;
  for (int i = 0; i < problem.active_rows; ++i) {
    const double residual = problem.original[i].g.dot(candidate) - problem.original[i].bound;
    if (!std::isfinite(residual) || residual > 0)
      return false;
  }
  return true;
}
}  // namespace
SafetyFilter::SafetyFilter(const Configuration& c)
    : config_(c), builder_(c), problem_(c) {
  if (c.recovery_torque_method == RecoveryTorqueMethod::native_qp)
    solver_ = make_solver(problem_.config, problem_);
}
const Result& SafetyFilter::compute(const ModelView& m, const Vec7& nominal,
                                    const TorqueBounds& box, double dt, uint64_t now,
                                    const PeerSnapshot* peer) noexcept {
  prepare(m, box, dt, now, peer);
  return solve_prepared(nominal);
}
Status SafetyFilter::prepare(const ModelView& m, const TorqueBounds& box, double dt, uint64_t now,
                             const PeerSnapshot* peer, const PeerExchange* exchange,
                             bool peer_avoidance) noexcept {
  preparation_ns_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  preparation_age_us_ = now >= m.timestamp_ns ? (now - m.timestamp_ns) * 1e-3 : 1e30;
  command_ready_ = false;
  source_ns_ = m.timestamp_ns;
  result_ = Result{};
  result_.backend = problem_.config.backend;
  bounds_ = box;
  const auto& set = builder_.build(m, dt, now, &bounds_, peer, exchange, peer_avoidance);
  reference_normals_.count = builder_.reference_normals.count;
  for (int i = 0; i < reference_normals_.count; ++i)
    reference_normals_.normals[i] = builder_.reference_normals.normals[i];
  // Previous binding normals only supervise reference windup; all actual torque
  // constraints are rebuilt from the fresh model, independently of these hints.
  for (int i = 0;
       peer_avoidance && i < previous_binding_.count && reference_normals_.count < kMaxRows; ++i)
    reference_normals_.normals[reference_normals_.count++] = previous_binding_.normals[i];
  result_.status = set.status;
  result_.rows = set.count;
  result_.group_pruned_pairs = builder_.group_pruned_pairs;
  result_.evaluated_peer_pairs = builder_.evaluated_peer_pairs;
  prepared_ = true;
  // Available even when the owner rejects preparation before nominal control
  // or solve_prepared(). Later finish() replaces these with full timings.
  result_.model_us = builder_.model_us;
  result_.kinematics_us = builder_.kinematics_us;
  result_.constraints_us = builder_.constraints_us;
  result_.total_us = us(Clock::time_point(std::chrono::nanoseconds(preparation_ns_)));
  return set.status;
}
const Result& SafetyFilter::solve_prepared(const Vec7& nominal) noexcept {
  const auto start = Clock::time_point(std::chrono::nanoseconds(preparation_ns_));
  if (!prepared_) {
    command_ready_ = false;
    result_.status = Status::invalid_model;
    return result_;
  }
  prepared_ = false;
  if (preparation_age_us_ + us(start) > problem_.config.max_model_age_us) {
    result_.status = Status::invalid_model;
    result_.total_us = us(start);
    return result_;
  }
  if (result_.status == Status::solved_hard) {
    if (!nominal.allFinite()) {
      result_.status = Status::invalid_model;
      finish();
      return result_;
    }
    if (config_.recovery_torque_method == RecoveryTorqueMethod::native_qp) {
      solve_recovery_qp(nominal);
      finish();
      return result_;
    }
    const auto& rows = builder_.current();
    const auto projection_start = Clock::now();
    const Vec7 candidate = recovery_projection(rows, nominal, bounds_, config_.weights,
                                               config_.recovery_projection_sweeps);
    result_.solve_us = us(projection_start);
    if (!checked_candidate(candidate, nominal, Status::verified_recovery) &&
        !checked_candidate(rows.braking_anchor, nominal, Status::verified_recovery))
      result_.status = Status::constraint_violation;
    result_.analytic_solution = true;
    finish();
    return result_;
  }
  finish();
  return result_;
}
Result SafetyFilter::solve_qp(const Vec7& nominal, const double* bound_override) noexcept {
  Result result;
  result.status = Status::solved_hard;
  if (result.status == Status::solved_hard) {
    const auto assembly = Clock::now();
    const auto status = problem_.assemble(builder_.current().rows.data(), builder_.current().count,
                                          nominal, bounds_, bound_override);
    const double assembly_us = us(assembly);
    result.status = status;
    if (status == Status::solved_hard) {
      // The actuator-box projection is the exact minimizer of the diagonal
      // torque objective. Feasible original rows make it the full QP optimum,
      // including when the raw controller law required clipping.
      Vec7 bounded_nominal = nominal.cwiseMax(bounds_.lower).cwiseMin(bounds_.upper);
      bool analytical = problem_.config.analytic_nominal && bounded_nominal.allFinite();
      for (int i = 0; analytical && i < problem_.active_rows; ++i)
        analytical = problem_.original[i].g.dot(bounded_nominal) <= problem_.original[i].bound;
      if (!analytical && problem_.config.analytic_nominal && bounded_nominal.allFinite())
        analytical = project_one_hard_row(problem_, nominal, bounds_, bounded_nominal);
      SolverRun raw;
      if (analytical) {
        problem_.solution.setZero();
        problem_.solution.head<7>() = bounded_nominal;
        raw.status = Status::solved_hard;
      } else
        raw = solver_->solve(problem_);
      const auto verify = Clock::now();
      result = verify_solution(problem_, raw.status, raw.iterations, nominal);
      result.analytic_solution = analytical;
      result.verify_us = us(verify);
      result.update_us = raw.update_us;
      result.solve_us = raw.solve_us;
    }
    result.assembly_us = assembly_us;
  }
  return result;
}
void SafetyFilter::solve_recovery_qp(const Vec7& nominal) noexcept {
  const auto& rows = builder_.current();
  auto solved = solve_qp(nominal, rows.requested_bounds.data());
  const Status requested_status = solved.status;
  Status recovery_status = Status::disabled;
  bool recovery_bounds = false;
  double assembly_us = solved.assembly_us, update_us = solved.update_us, solve_us = solved.solve_us,
         verify_us = solved.verify_us;
  int iterations = solved.iterations;
  // Only explicit infeasibility selects a second QP with the existing recovery
  // bounds. Iteration/numerical failures are reported separately, never relabeled
  // as physical infeasibility. A verified projection remains the bounded backup.
  if (solved.status == Status::infeasible) {
    solved = solve_qp(nominal);
    recovery_status = solved.status;
    recovery_bounds = true;
    assembly_us += solved.assembly_us;
    update_us += solved.update_us;
    solve_us += solved.solve_us;
    verify_us += solved.verify_us;
    iterations += solved.iterations;
  }
  bool accepted = false;
  if (solved.accepted()) {
    const Vec7 candidate = solved.torque.cwiseMax(bounds_.lower).cwiseMin(bounds_.upper);
    bool requested_ok = true;
    for (int i = 0; !recovery_bounds && requested_ok && i < rows.count; ++i)
      requested_ok =
          rows.rows[i].g.dot(candidate) <= rows.requested_bounds[i] + config_.hard_tolerance;
    accepted = requested_ok && checked_candidate(candidate, nominal, Status::verified_recovery);
    if (!accepted) {
      if (recovery_bounds)
        recovery_status = Status::constraint_violation;
    }
  }
  const bool fallback = !accepted && solved.status != Status::invalid_model;
  if (fallback) {
    const auto start = Clock::now();
    const Vec7 candidate = recovery_projection(rows, nominal, bounds_, config_.weights,
                                               config_.recovery_projection_sweeps);
    solve_us += us(start);
    accepted = checked_candidate(candidate, nominal, Status::verified_recovery) ||
               checked_candidate(rows.braking_anchor, nominal, Status::verified_recovery);
  }
  if (!accepted)
    result_.status = solved.status == Status::invalid_model ? Status::invalid_model
                                                            : Status::constraint_violation;
  result_.analytic_solution = !fallback && solved.analytic_solution;
  result_.requested_qp_status = requested_status;
  result_.recovery_qp_status = recovery_status;
  result_.recovery_projection_fallback = fallback;
  result_.assembly_us = assembly_us;
  result_.update_us = update_us;
  result_.solve_us = solve_us;
  result_.verify_us = verify_us;
  result_.iterations = iterations;
}

bool SafetyFilter::checked_candidate(const Vec7& torque, const Vec7& nominal,
                                     Status status) noexcept {
  const double box_tolerance = 0.;
  if (!torque.allFinite() || !nominal.allFinite() || !bounds_.lower.allFinite() ||
      !bounds_.upper.allFinite() || (bounds_.lower.array() > bounds_.upper.array()).any() ||
      (torque.array() < bounds_.lower.array() - box_tolerance).any() ||
      (torque.array() > bounds_.upper.array() + box_tolerance).any())
    return false;
  const auto& rows = builder_.current();
  const double tolerance = status == Status::solved_hard ? 0. : config_.hard_tolerance;
  for (int i = 0; i < rows.count; ++i)
    if (rows.rows[i].g.dot(torque) - rows.rows[i].bound > tolerance)
      return false;
  result_.status = status;
  result_.torque = torque;
  result_.rows = rows.count;
  result_.max_hard_violation = 0;
  result_.intervention_norm = (torque - nominal).norm();
  for (int i = 0; i < rows.count; ++i) {
    const auto& row = rows.rows[i];
    const double residual = row.g.dot(torque) - row.bound;
    result_.original_residual[i] = residual;
    result_.families[i] = row.family;
    result_.max_hard_violation = std::max(result_.max_hard_violation, residual);
  }
  return true;
}
void SafetyFilter::finish() noexcept {
  const auto start = Clock::time_point(std::chrono::nanoseconds(preparation_ns_));
  previous_binding_.count = 0;
  if (result_.accepted() && result_.intervention_norm > 1e-5)
    for (int i = 0; i < builder_.current().count; ++i) {
      const auto& row = builder_.current().rows[i];
      if (row.reference_normal.squaredNorm() > 1e-16 &&
          row.g.dot(result_.torque) - row.bound >= -10 * config_.hard_tolerance)
        previous_binding_.normals[previous_binding_.count++] = row.reference_normal;
    }
  const auto& rows = builder_.current();
  result_.recovery_rows = rows.recovery_rows;
  result_.outside_domain_rows = rows.outside_domain_rows;
  if (result_.accepted())
    for (int i = 0; i < rows.count; ++i)
      result_.max_requested_violation =
          std::max(result_.max_requested_violation,
                   rows.rows[i].g.dot(result_.torque) - rows.requested_bounds[i]);
  result_.model_us = builder_.model_us;
  result_.group_pruned_pairs = builder_.group_pruned_pairs;
  result_.evaluated_peer_pairs = builder_.evaluated_peer_pairs;
  result_.kinematics_us = builder_.kinematics_us;
  result_.constraints_us = builder_.constraints_us;
  result_.total_us = us(start);
  if (result_.total_us > config_.max_filter_time_us) {
    result_.status = Status::timing_overrun;
  }
  command_ready_ = result_.accepted();
}
CommandInspection SafetyFilter::inspect_command(const Vec7& torque) const noexcept {
  CommandInspection out;
  const auto& set = builder_.current();
  out.domain_valid = source_ns_ && set.status == Status::solved_hard;
  out.finite = torque.allFinite();
  if (!out.domain_valid || !out.finite ||
      !bounds_.lower.allFinite() || !bounds_.upper.allFinite() ||
      (bounds_.lower.array() > bounds_.upper.array()).any())
    return out;
  out.max_actuator_violation = std::max(
      0., std::max((bounds_.lower - torque).maxCoeff(), (torque - bounds_.upper).maxCoeff()));
  if (!std::isfinite(out.max_actuator_violation)) {
    out.finite = false;
    out.max_actuator_violation = 0;
    return out;
  }
  out.actuator_feasible = out.max_actuator_violation == 0;
  for (int i = 0; i < set.count; ++i) {
    const auto& row = set.rows[i];
    const double residual = row.g.dot(torque) - row.bound;
    if (!std::isfinite(residual)) {
      out.finite = false;
      return out;
    }
    if (residual > out.max_row_violation) {
      out.max_row_violation = residual;
      out.worst_family = static_cast<int>(row.family);
    }
    // A positive minimum is a witness that this original row cannot be met by
    // ANY torque in the current actuator interval, regardless of the solver.
    const double minimum =
        row.g.cwiseMax(0.).dot(bounds_.lower) + row.g.cwiseMin(0.).dot(bounds_.upper) - row.bound;
    if (!std::isfinite(minimum)) {
      out.finite = false;
      return out;
    }
    if (minimum > config_.hard_tolerance)
      ++out.impossible_rows;
    if (minimum > out.max_box_min_row_violation) {
      out.max_box_min_row_violation = minimum;
      out.impossible_family = static_cast<int>(row.family);
    }
  }
  out.rows_feasible = out.max_row_violation <= config_.hard_tolerance;
  return out;
}
bool SafetyFilter::verify_command(const Vec7& torque) const noexcept {
  if (!command_ready_ || !result_.accepted() || !torque.allFinite() ||
      (torque.array() < bounds_.lower.array()).any() ||
      (torque.array() > bounds_.upper.array()).any())
    return false;
  for (int r = 0; r < builder_.current().count; ++r) {
    const auto& row = builder_.current().rows[r];
    const double violation = row.g.dot(torque) - row.bound;
    if (violation > problem_.config.hard_tolerance)
      return false;
  }
  return true;
}
bool SafetyFilter::hold_correction_is_local(const Vec7&) const noexcept {
  if (!command_ready_ || !result_.accepted())
    return false;
  for (int i = 0; i < builder_.current().count; ++i)
    if (builder_.current().rows[i].family == Family::inter_arm)
      return false;
  return verify_command(result_.torque);
}
}  // namespace cbf_safety_filter
