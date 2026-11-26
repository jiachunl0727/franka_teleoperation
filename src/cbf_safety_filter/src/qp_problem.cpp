#include "cbf_safety_filter/qp_problem.hpp"

namespace cbf_safety_filter {
QpProblem::QpProblem(const Configuration& c) : config(c) {
  c.validate();
  hessian = Eigen::MatrixXd::Zero(7, 7);
  inequality = Eigen::MatrixXd::Zero(c.max_constraints, 7);
  gradient = Eigen::VectorXd::Zero(7);
  upper = Eigen::VectorXd::Ones(c.max_constraints);
  lower_box = Eigen::VectorXd::Zero(7);
  upper_box = Eigen::VectorXd::Zero(7);
  solution = Eigen::VectorXd::Zero(7);
}

Status QpProblem::assemble(const Row* rows, int count, const Vec7& nominal,
                           const TorqueBounds& box, const double* bound_override) noexcept {
  if (count < 0 || count > config.max_constraints)
    return Status::capacity_exceeded;
  if (!nominal.allFinite() || !box.lower.allFinite() || !box.upper.allFinite() ||
      (box.lower.array() > box.upper.array()).any() || (count && rows == nullptr))
    return Status::invalid_model;

  active_rows = count;
  hessian.setZero();
  hessian.diagonal() = config.weights;
  gradient = -config.weights.cwiseProduct(nominal);
  inequality.setZero();
  upper.setOnes();
  lower_box = box.lower;
  upper_box = box.upper;
  for (int r = 0; r < count; ++r) {
    original[r] = rows[r];
    if (bound_override)
      original[r].bound = bound_override[r];
    const auto& row = original[r];
    if (!row.g.allFinite() || !std::isfinite(row.bound))
      return Status::invalid_model;

    // Equilibrate geometric rows only in solver coordinates. Verification uses
    // each row in its original physical units.
    const bool geometric = row.family == Family::self_collision ||
                           row.family == Family::inter_arm || row.family == Family::singularity ||
                           row.family == Family::peer_motion;
    const double scale = geometric && row.g.norm() > 1e-10 ? 1. / row.g.norm() : 1.;
    inequality.row(r) = scale * row.g.transpose();
    upper[r] = scale * row.bound;
  }
  return Status::solved_hard;
}

bool QpProblem::valid() const noexcept {
  return hessian.rows() == 7 && hessian.cols() == 7 && gradient.size() == 7 &&
         solution.size() == 7 && inequality.rows() == config.max_constraints &&
         inequality.cols() == 7 && upper.size() == config.max_constraints &&
         lower_box.size() == 7 && upper_box.size() == 7 && active_rows >= 0 &&
         active_rows <= config.max_constraints && hessian.isDiagonal(0) && hessian.allFinite() &&
         gradient.allFinite() && inequality.allFinite() && upper.allFinite() &&
         lower_box.allFinite() && upper_box.allFinite() &&
         (lower_box.array() <= upper_box.array()).all() &&
         (hessian.diagonal().array() > 0).all();
}

Result verify_solution(const QpProblem& p, Status solver_status, int iterations,
                       const Vec7& nominal) noexcept {
  Result result;
  result.backend = p.config.backend;
  result.status = solver_status;
  result.iterations = iterations;
  result.rows = p.active_rows;
  if (!p.solution.allFinite() || !p.valid()) {
    result.status = Status::numerical_failure;
    return result;
  }
  result.torque = p.solution;
  result.intervention_norm = (result.torque - nominal).norm();
  result.max_hard_violation = std::max(
      0., std::max((p.lower_box - p.solution).maxCoeff(), (p.solution - p.upper_box).maxCoeff()));
  for (int r = 0; r < p.active_rows; ++r) {
    const double residual = p.original[r].g.dot(result.torque) - p.original[r].bound;
    result.original_residual[r] = residual;
    result.families[r] = p.original[r].family;
    result.max_hard_violation = std::max(result.max_hard_violation, residual);
  }
  if (solver_status != Status::solved_hard)
    return result;
  if (result.max_hard_violation > p.config.hard_tolerance)
    result.status = Status::constraint_violation;
  return result;
}
}  // namespace cbf_safety_filter
