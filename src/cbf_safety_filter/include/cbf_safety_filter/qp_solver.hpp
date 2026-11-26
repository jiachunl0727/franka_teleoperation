#pragma once
#include <memory>

#include "cbf_safety_filter/qp_problem.hpp"
namespace cbf_safety_filter {
struct SolverRun {
  Status status{Status::numerical_failure};
  int iterations{0};
  double update_us{0}, solve_us{0};
};
class QpSolver {
 public:
  virtual ~QpSolver() = default;
  virtual SolverRun solve(QpProblem&) noexcept = 0;
};
// Startup-only factory; no substitution when a backend fails. All implementations
// remain hardware-unqualified until allocation and complete callback release gates.
std::unique_ptr<QpSolver> make_solver(const Configuration&, const QpProblem&);
std::unique_ptr<QpSolver> make_osqp(const Configuration&, const QpProblem&);
std::unique_ptr<QpSolver> make_qpoases(const Configuration&, const QpProblem&);
std::unique_ptr<QpSolver> make_piqp(const Configuration&, const QpProblem&);
}  // namespace cbf_safety_filter
