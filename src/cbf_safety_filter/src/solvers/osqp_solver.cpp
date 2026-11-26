#include <osqp.h>

#include <chrono>
#include <stdexcept>
#include <vector>

#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf_safety_filter {
namespace {
using Clock = std::chrono::steady_clock;
double us(Clock::time_point t) {
  return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}
class Osqp final : public QpSolver {
  int n_, m_, rows_;
  std::vector<c_int> ap_, ai_, pp_, pi_;
  std::vector<c_float> ax_, px_, q_, l_, u_;
  csc a_{}, p_{};
  OSQPWorkspace* work_{nullptr};
  void update(const QpProblem& p) noexcept {
    for (int j = 0; j < n_; ++j) {
      px_[j] = p.hessian(j, j);
      q_[j] = p.gradient[j];
      int offset = ap_[j];
      for (int r = 0; r < rows_; ++r)
        ax_[offset + r] = p.inequality(r, j);
      ax_[offset + rows_] = 1;
      l_[rows_ + j] = p.lower_box[j];
      u_[rows_ + j] = p.upper_box[j];
    }
    for (int r = 0; r < rows_; ++r) {
      l_[r] = -OSQP_INFTY;
      u_[r] = p.upper[r];
    }
  }

 public:
  Osqp(const Configuration& c, const QpProblem& qp)
      : n_(qp.gradient.size()),
        m_(qp.upper.size() + n_),
        rows_(qp.upper.size()),
        ap_(n_ + 1),
        ai_(n_ * (rows_ + 1)),
        pp_(n_ + 1),
        pi_(n_),
        ax_(ai_.size()),
        px_(n_),
        q_(n_),
        l_(m_),
        u_(m_) {
    for (int j = 0; j < n_; ++j) {
      ap_[j] = j * (rows_ + 1);
      pp_[j] = j;
      pi_[j] = j;
      for (int r = 0; r < rows_; ++r)
        ai_[ap_[j] + r] = r;
      ai_[ap_[j] + rows_] = rows_ + j;
    }
    ap_[n_] = ax_.size();
    pp_[n_] = n_;
    a_.m = m_;
    a_.n = n_;
    a_.nz = -1;
    a_.nzmax = ax_.size();
    a_.p = ap_.data();
    a_.i = ai_.data();
    a_.x = ax_.data();
    p_.m = p_.n = n_;
    p_.nz = -1;
    p_.nzmax = n_;
    p_.p = pp_.data();
    p_.i = pi_.data();
    p_.x = px_.data();
    update(qp);
    OSQPData data{};
    data.n = n_;
    data.m = m_;
    data.P = &p_;
    data.A = &a_;
    data.q = q_.data();
    data.l = l_.data();
    data.u = u_.data();
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.max_iter = c.osqp_iterations;
    settings.eps_abs = c.osqp_abs;
    settings.eps_rel = c.osqp_rel;
    settings.warm_start = c.warm_start;
    settings.polish = 0;
    settings.adaptive_rho = 0;
    // Initial setup contains padded zero dynamic rows. Disable one-time scaling;
    // independent verification always uses unscaled original physical units.
    settings.scaling = 0;
    if (osqp_setup(&work_, &data, &settings) != 0)
      throw std::runtime_error("OSQP setup failed");
  }
  ~Osqp() override {
    if (work_)
      osqp_cleanup(work_);
  }
  SolverRun solve(QpProblem& p) noexcept override {
    SolverRun run;
    if (!p.valid()) {
      run.status = Status::invalid_model;
      return run;
    }
    const auto start = Clock::now();
    update(p);
    if (osqp_update_P_A(work_, px_.data(), nullptr, n_, ax_.data(), nullptr, ax_.size()) ||
        osqp_update_lin_cost(work_, q_.data()) || osqp_update_bounds(work_, l_.data(), u_.data()))
      return run;
    run.update_us = us(start);
    const auto solving = Clock::now();
    const int error = osqp_solve(work_);
    run.solve_us = us(solving);
    run.iterations = work_->info->iter;
    if (error)
      return run;
    const auto status = work_->info->status_val;
    if (status == OSQP_SOLVED)
      run.status = Status::solved_hard;
    else if (status == OSQP_MAX_ITER_REACHED)
      run.status = Status::iteration_limit;
    else if (status == OSQP_PRIMAL_INFEASIBLE || status == OSQP_PRIMAL_INFEASIBLE_INACCURATE)
      run.status = Status::infeasible;
    else
      run.status = Status::numerical_failure;
    if (work_->solution && work_->solution->x)
      for (int j = 0; j < n_; ++j)
        p.solution[j] = work_->solution->x[j];
    return run;
  }
};
}  // namespace
std::unique_ptr<QpSolver> make_osqp(const Configuration& c, const QpProblem& p) {
  return std::make_unique<Osqp>(c, p);
}
}  // namespace cbf_safety_filter
