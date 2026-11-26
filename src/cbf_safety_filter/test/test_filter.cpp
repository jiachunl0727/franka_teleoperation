#include <gtest/gtest.h>
#include "cbf_safety_filter/safety_filter.hpp"
#include "fixture.hpp"
namespace cbf = cbf_safety_filter;
TEST(Filter, FeasibleNominalIsExactBoundedOptimum) {
  cbf::Configuration c;
  c.position.enabled = c.velocity.enabled = true;
  c.max_filter_time_us = 1e6;
  cbf::SafetyFilter filter(c);
  cbf_test::Model m;
  cbf_test::refresh(m);
  const cbf::Vec7 nominal = (cbf::Vec7() << 2, -2, .01, 2, -2, 2, -2).finished();
  cbf::TorqueBounds box{cbf::Vec7::Constant(-.1), cbf::Vec7::Constant(.2)};
  box.lower[6] = box.upper[6] = .05;
  const auto expected = nominal.cwiseMax(box.lower).cwiseMin(box.upper);
  const auto& result = filter.compute(m.view(), nominal, box, .001, 1);
  ASSERT_TRUE(result.accepted()) << cbf::name(result.status);
  EXPECT_TRUE(result.analytic_solution);
  EXPECT_TRUE(result.torque.isApprox(expected, 0));
  EXPECT_TRUE(filter.verify_command(result.torque));
}
TEST(Filter, ProjectsNominalOntoJointBoundaryAndVerifiesCommand) {
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  c.position.enabled = c.velocity.enabled = true;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  cbf_test::Model m;
  m.q[0] = 2.8973 - c.position.margin - .01;
  cbf_test::refresh(m);
  cbf::Vec7 nominal = cbf::Vec7::Zero();
  nominal[0] = 1;
  cbf::SafetyFilter filter(c);
  const cbf::TorqueBounds box{-cbf::Vec7::Ones(), cbf::Vec7::Ones()};
  const auto result = filter.compute(m.view(), nominal, box, .001, 1);
  ASSERT_TRUE(result.accepted()) << cbf::name(result.status);
  EXPECT_NEAR(result.torque[0], .25, 1e-12);
  EXPECT_TRUE(filter.verify_command(result.torque));
}
TEST(Filter, RejectsInvalidModelAndDeadlineOverrun) {
  cbf::Configuration c;
  c.position.enabled = true;
  c.max_filter_time_us = 1e-12;
  cbf_test::Model m;
  cbf_test::refresh(m);
  cbf::TorqueBounds box{cbf::Vec7::Constant(-2), cbf::Vec7::Constant(2)};
  cbf::SafetyFilter timed(c);
  EXPECT_EQ(timed.compute(m.view(), cbf::Vec7::Zero(), box, .001, 1).status,
            cbf::Status::timing_overrun);
  c.max_filter_time_us = 1e8;
  cbf::SafetyFilter invalid(c);
  m.q[0] = 3;
  EXPECT_EQ(invalid.compute(m.view(), cbf::Vec7::Zero(), box, .001, 1).status,
            cbf::Status::invalid_domain);
}
