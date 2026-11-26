#pragma once
#include "cbf_safety_filter/constraints.hpp"
namespace cbf_safety_filter {
class QpProblem {
 public:
  explicit QpProblem(const Configuration&);
  Status assemble(const Row* rows, int count, const Vec7& nominal, const TorqueBounds&,
                  const double* bound_override = nullptr) noexcept;
  bool valid() const noexcept;
  Eigen::MatrixXd hessian, inequality;
  Eigen::VectorXd gradient, upper, lower_box, upper_box, solution;
  std::array<Row, kMaxRows> original{};
  int active_rows{0};
  const Configuration config;
};
Result verify_solution(const QpProblem&, Status solver_status, int iterations,
                       const Vec7& nominal) noexcept;
}  // namespace cbf_safety_filter
