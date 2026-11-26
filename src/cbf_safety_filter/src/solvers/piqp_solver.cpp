#include <chrono>
#include <piqp/piqp.hpp>

#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf_safety_filter {
namespace {
using Clock = std::chrono::steady_clock;
double us(Clock::time_point t) {
  return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}
class InteriorPoint final : public QpSolver {
  piqp::DenseSolver<double> solver_;

 public:
  InteriorPoint(const Configuration& c, const QpProblem& p) {
    solver_.settings().verbose = false;
    solver_.settings().compute_timings = false;
    solver_.settings().max_iter = c.piqp_iterations;
    solver_.settings().eps_abs = c.piqp_abs;
    solver_.settings().eps_rel = c.piqp_rel;
    solver_.setup(p.hessian, p.gradient, piqp::nullopt, piqp::nullopt, p.inequality, piqp::nullopt,
                  p.upper, p.lower_box, p.upper_box);
  }
  SolverRun solve(QpProblem& p) noexcept override {
    SolverRun run;
    if (!p.valid()) {
      run.status = Status::invalid_model;
      return run;
    }
    const auto update = Clock::now();
    solver_.update(p.hessian, p.gradient, piqp::nullopt, piqp::nullopt, p.inequality, piqp::nullopt,
                   p.upper, p.lower_box, p.upper_box);
    run.update_us = us(update);
    const auto start = Clock::now();
    const auto status = solver_.solve();
    run.solve_us = us(start);
    run.iterations = solver_.result().info.iter;
    if (status == piqp::PIQP_SOLVED)
      run.status = Status::solved_hard;
    else if (status == piqp::PIQP_MAX_ITER_REACHED)
      run.status = Status::iteration_limit;
    else if (status == piqp::PIQP_PRIMAL_INFEASIBLE)
      run.status = Status::infeasible;
    p.solution = solver_.result().x;
    return run;
  }
};
}  // namespace
std::unique_ptr<QpSolver> make_piqp(const Configuration& c, const QpProblem& p) {
  return std::make_unique<InteriorPoint>(c, p);
}
}  // namespace cbf_safety_filter
