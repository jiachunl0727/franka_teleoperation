#include <gtest/gtest.h>

#include "teleop_controllers/inverse_dynamics_controller.hpp"
#include "teleop_controllers/modal_damping.hpp"

namespace ft = teleop_controllers;

TEST(ModalDamping, DiagonalCaseHasIndependentCriticalDamping) {
  ft::Vector6 inertia_diagonal;
  inertia_diagonal << 2, 3, 4, .2, .3, .4;
  const ft::Matrix6 inertia = inertia_diagonal.asDiagonal();
  ft::Vector6 stiffness;
  stiffness << 750, 750, 750, 75, 75, 75;
  ft::Matrix6 damping;
  ASSERT_TRUE(ft::modal_damping(inertia, stiffness, 1., damping));
  const ft::Vector6 expected = 2 * (inertia.diagonal().array() * stiffness.array()).sqrt();
  EXPECT_TRUE(damping.isApprox(expected.asDiagonal().toDenseMatrix(), 1e-12));
}

TEST(ModalDamping, CoupledMassAndStiffnessHaveSpecifiedModalDamping) {
  ft::Matrix6 lower = ft::Matrix6::Identity();
  lower.diagonal() << 2, 1.8, 1.6, .5, .4, .3;
  lower(4, 0) = .3;
  lower(5, 2) = -.2;
  const ft::Matrix6 inertia = lower * lower.transpose();
  ft::Vector6 stiffness;
  stiffness << 750, 700, 650, 75, 70, 65;
  ft::Matrix6 damping;
  ASSERT_TRUE(ft::modal_damping(inertia, stiffness, .8, damping));
  const ft::Matrix6 k = stiffness.asDiagonal();
  const Eigen::GeneralizedSelfAdjointEigenSolver<ft::Matrix6> modes(k, inertia);
  const ft::Matrix6 v = modes.eigenvectors();
  EXPECT_TRUE((v.transpose() * inertia * v).isIdentity(1e-11));
  const ft::Matrix6 expected = (1.6 * modes.eigenvalues().array().sqrt()).matrix().asDiagonal();
  EXPECT_TRUE((v.transpose() * damping * v).isApprox(expected, 1e-11));
  EXPECT_TRUE(damping.isApprox(damping.transpose(), 1e-12));
  EXPECT_TRUE(Eigen::LLT<ft::Matrix6>(damping).info() == Eigen::Success);
}

TEST(ModalDamping, InvalidDynamicsAreRejected) {
  ft::Matrix6 damping;
  ft::Matrix6 inertia = ft::Matrix6::Identity();
  EXPECT_FALSE(ft::modal_damping(inertia, ft::Vector6::Zero(), 1., damping));
  EXPECT_FALSE(ft::modal_damping(inertia, ft::Vector6::Ones(), 0., damping));
  inertia(0, 0) = -1;
  EXPECT_FALSE(ft::modal_damping(inertia, ft::Vector6::Ones(), 1., damping));
}

namespace {
ft::ArmState coupled_state() {
  ft::ArmState s;
  s.q = Eigen::Map<const ft::Vector7>(teleop_controllers::kHome.data());
  s.q[0] += .2;
  s.dq << .1, -.2, .15, .05, -.03, .1, .2;
  s.coriolis << .1, .2, -.1, .05, -.03, .01, .02;
  s.jacobian.leftCols<6>().setIdentity();
  s.jacobian.col(6) << .4, -.2, .1, .3, .2, -.1;
  Eigen::Matrix<double, 7, 7> l = Eigen::Matrix<double, 7, 7>::Identity();
  l.diagonal() << 2, 1.8, 1.6, 1.4, 1.2, 1, .8;
  l(6, 0) = .3;
  l(5, 2) = -.2;
  s.mass = l * l.transpose();
  return s;
}
}  // namespace

TEST(InverseDynamics, MatchesOscbfStructureWithPhysicalStiffnessAndModalDamping) {
  auto s = coupled_state();
  ft::InverseDynamicsParameters p;
  ASSERT_TRUE(p.valid());
  ft::CartesianReference ref;
  ref.position << .01, -.02, .03;
  ref.orientation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ());
  ref.twist << .01, .02, -.01, .03, -.02, .01;
  ref.acceleration << .1, -.1, .2, -.2, .1, .05;
  ft::InverseDynamicsController law(p);
  const auto out = law.compute(s, ref, .001);
  ASSERT_EQ(out.fault, ft::StateFault::none);
  const auto minv = s.mass.inverse().eval();
  const ft::Matrix6 lambda = (s.jacobian * minv * s.jacobian.transpose()).inverse();
  const ft::Jacobian jbar_t = lambda * s.jacobian * minv;
  ft::Matrix6 damping;
  ASSERT_TRUE(ft::modal_damping(lambda, p.stiffness, p.damping_ratio, damping));
  ft::Vector6 error;
  error << -ref.position, 0, 0, -std::sin(.4);
  const ft::Vector6 task =
      ref.acceleration - lambda.inverse() * (p.stiffness.cwiseProduct(error) +
                                             damping * (s.jacobian * s.dq - ref.twist));
  const ft::Vector7 posture =
      p.nullspace_stiffness.cwiseProduct(p.common.home_joint_position - s.q) -
      p.nullspace_damping.cwiseProduct(s.dq);
  const ft::Vector7 expected =
      s.jacobian.transpose() * lambda * task + s.coriolis +
      (Eigen::Matrix<double, 7, 7>::Identity() - s.jacobian.transpose() * jbar_t) * s.mass *
          posture;
  EXPECT_TRUE(out.torque.isApprox(expected, 1e-11));
}

TEST(InverseDynamics, PreservesPhysicalStiffnessAndTaskAccelerationFeedforward) {
  auto s = coupled_state();
  s.dq.setZero();
  ft::InverseDynamicsParameters p;
  p.common.home_joint_position = s.q;
  ft::InverseDynamicsController law(p);
  ft::CartesianReference ref;
  ref.position.x() = .01;
  auto out = law.compute(s, ref, .001);
  ASSERT_EQ(out.fault, ft::StateFault::none);
  ft::Vector6 wrench = ft::Vector6::Zero();
  wrench[0] = 7.5;
  EXPECT_TRUE(out.torque.isApprox(s.jacobian.transpose() * wrench + s.coriolis, 1e-11));
  ref.position.setZero();
  ref.acceleration << .1, -.2, .3, -.1, .2, -.3;
  out = law.compute(s, ref, .001);
  EXPECT_TRUE(
      (s.jacobian * s.mass.llt().solve(out.torque - s.coriolis)).isApprox(ref.acceleration, 1e-11));
}

TEST(InverseDynamics, NullspaceTorqueDoesNotChangeTaskAcceleration) {
  auto s = coupled_state();
  ft::InverseDynamicsParameters p;
  p.common.home_joint_position = s.q;
  ft::InverseDynamicsController current_posture(p);
  p.common.home_joint_position[6] += .4;
  ft::InverseDynamicsController shifted_posture(p);
  const ft::CartesianReference ref;
  const ft::Vector7 secondary = shifted_posture.compute(s, ref, .001).torque -
                               current_posture.compute(s, ref, .001).torque;
  EXPECT_GT(secondary.norm(), .01);
  EXPECT_LT((s.jacobian * s.mass.llt().solve(secondary)).norm(), 1e-10);
}

TEST(InverseDynamics, InvalidModelAndRankLossReturnFaultWithoutFallback) {
  auto s = coupled_state();
  ft::InverseDynamicsParameters p;
  ft::InverseDynamicsController law(p);
  EXPECT_EQ(law.compute(s, ft::JointReference{}, .001).fault, ft::StateFault::invalid_reference);
  s.jacobian.row(5).setZero();
  EXPECT_EQ(law.compute(s, ft::CartesianReference{}, .001).fault, ft::StateFault::singular_task);
  s = coupled_state();
  s.mass(0, 0) = -1;
  EXPECT_NE(law.compute(s, ft::CartesianReference{}, .001).fault, ft::StateFault::none);
  p.stiffness[0] = -1;
  EXPECT_FALSE(p.valid());
}
