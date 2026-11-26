#include <stdexcept>

#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf_safety_filter {
std::unique_ptr<QpSolver> make_solver(const Configuration& c, const QpProblem& p) {
  c.validate();
  switch (c.backend) {
    case Backend::osqp:
      return make_osqp(c, p);
    case Backend::qpoases:
      return make_qpoases(c, p);
    case Backend::piqp:
      return make_piqp(c, p);
  }
  throw std::invalid_argument("unknown CBF solver backend");
}
}  // namespace cbf_safety_filter
