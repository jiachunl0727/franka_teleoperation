#include <chrono>
#include <qpOASES.hpp>
#include <stdexcept>

#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf_safety_filter {
namespace {
using Clock = std::chrono::steady_clock;
double us(Clock::time_point t) {
  return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}
class ActiveSet final : public QpSolver {
  Configuration config_;
  qpOASES::SQProblem solver_;
  // SQProblem retains shallow matrix references. Alternate two persistent banks
  // so matrix updates never overwrite its previous H/A before hotstart.
  using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  std::array<Matrix, 2> hessian_, matrix_;
  int bank_{0};
  Eigen::VectorXd lower_;
  bool initialized_{false};

 public:
  ActiveSet(const Configuration& c, const QpProblem& p)
      : config_(c),
        solver_(p.gradient.size(), p.upper.size(), qpOASES::HST_POSDEF),
        lower_(Eigen::VectorXd::Constant(p.upper.size(), -qpOASES::INFTY)) {
    for (int i = 0; i < 2; ++i) {
      hessian_[i] = p.hessian;
      matrix_[i] = p.inequality;
    }
    qpOASES::Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;
    options.terminationTolerance = c.qpoases_tolerance;
    options.enableFarBounds = qpOASES::BT_FALSE;
    solver_.setOptions(options);
    // Workspace bootstrap is outside control and may need to establish the
    // fixed box bounds. It has its own finite startup cap; the
    // configured per-cycle nWSR budget is applied literally in solve().
    int budget = 2000;
    const auto status =
        solver_.init(hessian_[0].data(), p.gradient.data(), matrix_[0].data(), p.lower_box.data(),
                     p.upper_box.data(), lower_.data(), p.upper.data(), budget);
    if (status != qpOASES::SUCCESSFUL_RETURN)
      throw std::runtime_error("qpOASES startup workspace initialization failed");
    initialized_ = true;
  }
  SolverRun solve(QpProblem& p) noexcept override {
    SolverRun run;
    if (!p.valid()) {
      run.status = Status::invalid_model;
      return run;
    }
    const auto updating = Clock::now();
    bank_ = 1 - bank_;
    hessian_[bank_] = p.hessian;
    matrix_[bank_] = p.inequality;
    run.update_us = us(updating);
    int budget = config_.qpoases_recalculations;
    const auto start = Clock::now();
    qpOASES::returnValue status;
    if (initialized_ && config_.warm_start)
      status = solver_.hotstart(hessian_[bank_].data(), p.gradient.data(), matrix_[bank_].data(),
                                p.lower_box.data(), p.upper_box.data(), lower_.data(),
                                p.upper.data(), budget);
    else {
      solver_.reset();
      status = solver_.init(hessian_[bank_].data(), p.gradient.data(), matrix_[bank_].data(),
                            p.lower_box.data(), p.upper_box.data(), lower_.data(), p.upper.data(),
                            budget);
    }
    run.solve_us = us(start);
    run.iterations = budget;
    initialized_ = status == qpOASES::SUCCESSFUL_RETURN;
    if (status == qpOASES::SUCCESSFUL_RETURN)
      run.status = Status::solved_hard;
    else if (status == qpOASES::RET_MAX_NWSR_REACHED)
      run.status = Status::iteration_limit;
    else if (solver_.isInfeasible() == qpOASES::BT_TRUE)
      run.status = Status::infeasible;
    if (initialized_)
      solver_.getPrimalSolution(p.solution.data());
    return run;
  }
};
}  // namespace
std::unique_ptr<QpSolver> make_qpoases(const Configuration& c, const QpProblem& p) {
  return std::make_unique<ActiveSet>(c, p);
}
}  // namespace cbf_safety_filter
