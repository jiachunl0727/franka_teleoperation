#include <gtest/gtest.h>
#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf = cbf_safety_filter;
class HardSolvers : public testing::TestWithParam<cbf::Backend> {};
TEST_P(HardSolvers, SolvesAndVerifiesHardRows) {
  cbf::Configuration c;
  c.backend = GetParam();
  c.max_constraints = 8;
  c.osqp_iterations = 20000;
  c.piqp_iterations = 200;
  c.qpoases_recalculations = 1000;
  c.osqp_abs = c.osqp_rel = c.piqp_abs = c.piqp_rel = c.qpoases_tolerance = 1e-9;
  cbf::QpProblem problem(c);
  auto solver = cbf::make_solver(c, problem);
  cbf::Row row;
  row.g[0] = 1;
  row.bound = -.5;
  const cbf::TorqueBounds box{cbf::Vec7::Constant(-2), cbf::Vec7::Constant(2)};
  ASSERT_EQ(problem.assemble(&row, 1, cbf::Vec7::Zero(), box), cbf::Status::solved_hard);
  auto raw = solver->solve(problem);
  const auto result = cbf::verify_solution(problem, raw.status, raw.iterations, cbf::Vec7::Zero());
  ASSERT_TRUE(result.accepted()) << cbf::name(result.status);
  EXPECT_NEAR(result.torque[0], -.5, 2e-6);
  EXPECT_LE(result.max_hard_violation, c.hard_tolerance);
}
INSTANTIATE_TEST_SUITE_P(AllBackends, HardSolvers,
                         testing::Values(cbf::Backend::osqp, cbf::Backend::qpoases,
                                         cbf::Backend::piqp));
TEST(QpVerification, RejectsViolatedAndNonfiniteSolutions) {
  cbf::Configuration c;
  c.max_constraints = 1;
  cbf::QpProblem problem(c);
  cbf::Row row;
  row.g[0] = 1;
  row.bound = -1;
  cbf::TorqueBounds box{cbf::Vec7::Constant(-2), cbf::Vec7::Constant(2)};
  ASSERT_EQ(problem.assemble(&row, 1, cbf::Vec7::Zero(), box), cbf::Status::solved_hard);
  problem.solution.setZero();
  EXPECT_EQ(cbf::verify_solution(problem, cbf::Status::solved_hard, 1,
                                 cbf::Vec7::Zero()).status,
            cbf::Status::constraint_violation);
  problem.solution[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(cbf::verify_solution(problem, cbf::Status::solved_hard, 1,
                                 cbf::Vec7::Zero()).status,
            cbf::Status::numerical_failure);
}
