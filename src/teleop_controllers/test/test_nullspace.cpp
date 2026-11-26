#include <gtest/gtest.h>

#include "teleop_controllers/cartesian_pd_controller.hpp"
#include "teleop_controllers/orientation_only_controller.hpp"
namespace ft = teleop_controllers;
namespace {
ft::ArmState state() {
  ft::ArmState s;
  s.q = Eigen::Map<const ft::Vector7>(teleop_controllers::kHome.data());
  s.q[0] += .3;
  s.q[6] -= .2;
  s.jacobian << 1, 0, 0, 0, 0, 0, .4, 0, 1, 0, 0, 0, 0, .2, 0, 0, 1, 0, 0, 0, -.3, 0, 0, 0, 1, 0, 0,
      .1, 0, 0, 0, 0, 1, 0, .5, 0, 0, 0, 0, 0, 1, .3;
  Eigen::Matrix<double, 7, 7> l = Eigen::Matrix<double, 7, 7>::Identity();
  l.diagonal() << 2, 1.8, 1.6, 1.4, 1.2, 1, .8;
  l(6, 0) = .3;
  l(5, 2) = -.2;
  s.mass = l * l.transpose();
  return s;
}
ft::CartesianReference reference(const ft::ArmState& s) {
  return {s.position, s.orientation, ft::Vector6::Zero()};
}
}  // namespace
TEST(Nullspace, DynamicProjectionPreservesTaskAccelerationWithNonidentityMass) {
  auto s = state();
  ft::CartesianPdParameters p;
  p.nullspace_ramp_seconds = .001;
  ft::CartesianPdController law(p);
  auto out = law.compute(s, reference(s), .001);
  ASSERT_EQ(out.fault, ft::StateFault::none);
  ASSERT_GT(out.torque.norm(), .01);
  EXPECT_LT((s.jacobian * s.mass.ldlt().solve(out.torque)).norm(), 1e-10);
  // A geometric torque projection would not in general pass the check above.
  const ft::Vector7 bias = p.nullspace_stiffness * (p.home_joint_position - s.q);
  EXPECT_GT(out.torque.dot(bias), 0);
}
TEST(Nullspace, UniformCapAndRankLossStayFinite) {
  auto s = state();
  s.jacobian.row(5).setZero();
  ft::CartesianPdParameters p;
  p.nullspace_max_torque = .05;
  p.nullspace_ramp_seconds = .001;
  ft::CartesianPdController law(p);
  const auto out = law.compute(s, reference(s), .001);
  EXPECT_TRUE(out.torque.allFinite());
  EXPECT_LE(out.torque.cwiseAbs().maxCoeff(), .05 + 1e-12);
  EXPECT_LT((s.jacobian * s.mass.ldlt().solve(out.torque)).norm(), 1e-10);
}
TEST(Nullspace, RampResetsInBothCartesianModesAndUsesConfiguredHome) {
  auto s = state();
  ft::CartesianPdParameters p;
  p.home_joint_position = s.q;
  p.home_joint_position[6] += .4;
  ASSERT_TRUE(p.valid());
  ft::CartesianPdController law(p);
  ft::OrientationOnlyController orientation(p);
  const auto first = law.compute(s, reference(s), .001).torque;
  for (int i = 0; i < 300; ++i)
    law.compute(s, reference(s), .001);
  EXPECT_GT(law.compute(s, reference(s), .001).torque.norm(), 100 * first.norm());
  law.reset(s);
  EXPECT_TRUE(law.compute(s, reference(s), .001).torque.isApprox(first));
  for (int i = 0; i < 300; ++i)
    orientation.compute(s, reference(s), .001);
  orientation.reset(s);
  EXPECT_TRUE(orientation.compute(s, reference(s), .001).torque.isApprox(first));
}
TEST(Nullspace, LimitRepulsionAndInvalidMassAreHandled) {
  ft::ArmState s = state();
  s.mass.setIdentity();
  s.jacobian.setZero();
  s.q[0] = teleop_controllers::kJointLower[0] + .01;
  ft::CartesianPdParameters p;
  p.home_joint_position = s.q;
  p.nullspace_ramp_seconds = .001;
  ft::CartesianPdController law(p);
  EXPECT_GT(law.compute(s, reference(s), .001).torque[0], 0);
  s.mass(0, 0) = -1;
  EXPECT_NE(law.compute(s, reference(s), .001).fault, ft::StateFault::none);
  p.home_joint_position[0] = teleop_controllers::kJointLower[0];
  EXPECT_FALSE(p.valid());
}

TEST(CartesianEffort, LargePoseAndVelocityErrorsHaveBoundedWrench) {
  ft::CartesianPdParameters p;
  ft::CartesianPdController unlimited(p);
  EXPECT_FALSE(p.cartesian_wrench_limits_enabled);
  p.cartesian_wrench_limits_enabled = true;
  ft::CartesianPdController law(p);
  ft::ArmState s;
  s.jacobian.leftCols<6>().setIdentity();
  auto target = reference(s);
  target.position.setConstant(10);
  target.orientation = Eigen::AngleAxisd(2., Eigen::Vector3d::UnitZ());
  target.twist.setConstant(100.);
  const auto raw = unlimited.compute(s, target, .001);
  EXPECT_GT(raw.torque.head<3>().norm(), p.cartesian_force_limit);
  EXPECT_GT(raw.torque.segment<3>(3).norm(), p.cartesian_moment_limit);
  const auto out = law.compute(s, target, .001);
  EXPECT_LE(out.torque.head<3>().norm(), p.cartesian_force_limit + 1e-12);
  EXPECT_LE(out.torque.segment<3>(3).norm(), p.cartesian_moment_limit + 1e-12);
  p.cartesian_force_limit = 0;
  EXPECT_FALSE(p.valid());
}
