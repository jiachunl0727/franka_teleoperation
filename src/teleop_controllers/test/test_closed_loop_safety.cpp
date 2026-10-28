// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#include <gtest/gtest.h>

#include "safety_filter/async_safety.hpp"
#include "safety_filter/panda_model_fixture.hpp"
#include "teleop_controllers/cartesian_impedance_controller.hpp"

namespace sf = safety_filter;
namespace ft = teleop_controllers;

class ClosedLoopSafety : public testing::TestWithParam<int> {};

TEST_P(ClosedLoopSafety, CartesianStepMovesWithCoupledInertiaWorkerAndFilteredTorqueFeedback) {
  // Synthetic, gravity-compensated plant, not a Panda dynamics model. Unlike
  // static feasibility fixtures, q/dq, geometry and derivative history evolve
  // from the returned torque. The actual controller/worker/local guard run.
  sf::cbf::SafetyParameters safety;
  safety.active_arms = {true, false};
  safety.singularity_enabled = true;
  sf::CoordinatorParameters limits;
  limits.asynchronous = true;
  std::array<sf::cbf::ArmIdentity, 2> ids;
  ids[0].arm_id = "left";
  ids[0].base_frame = "left/base";
  ids[1].arm_id = "right";
  ids[1].base_frame = "right/base";
  ids[1].world_from_base.translation().x() = 2;
  sf::cbf::CbfSafetyFilter worker;
  sf::AsyncSafetyController local;
  std::string error;
  ASSERT_TRUE(worker.configure(safety, ids, &error)) << error;
  ASSERT_TRUE(local.configure(0, limits, safety, ids, &error)) << error;
  ft::CartesianImpedanceParameters gains;
  gains.lambda_translation = gains.lambda_rotation = 15;
  gains.gain_translation = 50;
  gains.gain_rotation = 5;
  gains.nullspace_enabled = true;
  ft::CartesianImpedanceController controller(gains);
  auto q = gains.home_joint_position.eval();
  ft::Vector7 dq = ft::Vector7::Zero(), previous = dq;
  const auto initial = sf::panda_model_fixture(q).frame_pose[8];
  ft::CartesianReference target{initial.translation(), Eigen::Quaterniond(initial.linear()),
                                ft::Vector6::Zero()};
  const int axis = (GetParam() % 6) / 2;
  const double sign = GetParam() % 2 ? -1. : 1.;
  if (GetParam() < 6)
    target.position[axis] += sign * .02;
  else
    target.orientation =
        Eigen::AngleAxisd(sign * .05, Eigen::Vector3d::Unit(axis)) * target.orientation;
  const auto tracking_error = [&](const Eigen::Affine3d& pose) {
    return (target.position - pose.translation()).norm() +
           .3 *
               Eigen::AngleAxisd(target.orientation.conjugate() * Eigen::Quaterniond(pose.linear()))
                   .angle();
  };
  sf::SafetyReply reply;
  const double initial_error = tracking_error(initial);
  double final_error = initial_error;
  for (int cycle = 1; cycle <= 2000; ++cycle) {
    sf::ModelRequest request;
    request.cycle = cycle;
    request.stamp_ns = 1000000000ULL + cycle * 1000000ULL;
    request.epoch = 1;
    request.motion_active = true;
    request.state = sf::panda_model_fixture(q);
    auto& model = request.state;
    model.dq = dq;
    model.previous_torque = previous;
    sf::cbf::Matrix7d diagonal = sf::cbf::Matrix7d::Zero();
    diagonal.diagonal() << 2.5, 2, 1.2, .7, .03, .02, .006;
    sf::cbf::Matrix7d coupling = sf::cbf::Matrix7d::Identity();
    for (int i = 1; i < 7; ++i)
      for (int j = 0; j < i; ++j)
        coupling(i, j) = .3 * std::sin(i * 2 + j * 3 + .001 * cycle);
    model.mass = coupling * diagonal * coupling.transpose();
    ASSERT_TRUE(local.prepare_model(model, request.stamp_ns));
    ft::ArmState measured;
    measured.q = q;
    measured.dq = dq;
    measured.mass = model.mass;
    measured.position = model.frame_pose[8].translation();
    measured.orientation = Eigen::Quaterniond(model.frame_pose[8].linear());
    measured.jacobian = model.frame_jacobian[8];
    const auto nominal = controller.compute(measured, target, .001);
    ASSERT_EQ(nominal.fault, ft::StateFault::none);
    request.nominal = nominal.torque;
    const sf::SafetyReply* incoming = nullptr;
    if (cycle % 5 == 1) {
      sf::cbf::DualArmState state;
      state.arm[0] = model;
      sf::cbf::Vector14d torque = sf::cbf::Vector14d::Zero();
      torque.head<7>() = nominal.torque;
      reply = {};
      reply.cycles[0] = cycle;
      reply.source_ns[0] = reply.completed_ns = request.stamp_ns;
      reply.epoch[0] = request.epoch;
      reply.source_q[0] = q;
      reply.source_dq[0] = dq;
      reply.nominal = torque;
      reply.result = worker.filter(torque, state, .005, {true, false});
      ASSERT_TRUE(reply.result.solver_succeeded) << cycle << ": " << reply.result.solver_status;
      incoming = &reply;
    }
    const auto out = local.update(request, nullptr, incoming, request.stamp_ns);
    ASSERT_FALSE(out.degraded) << cycle << ": " << out.degraded_cause << " / "
                               << out.reply.result.solver_status;
    ASSERT_FALSE(out.reply.result.fatal_solver_fault);
    EXPECT_LE(out.reply.result.maximum_constraint_violation, safety.solver_feasibility_tolerance);
    const auto commanded = out.reply.result.safe_torque.head<7>().eval();
    EXPECT_LE((commanded - previous).cwiseAbs().maxCoeff(), .35000001);
    previous = .4 * commanded + .6 * previous;
    const ft::Vector7 acceleration = model.mass.ldlt().solve(previous);
    q += .001 * dq + .5e-6 * acceleration;
    dq += .001 * acceleration;
    final_error = tracking_error(sf::panda_model_fixture(q).frame_pose[8]);
  }
  // Require substantial progress on each axis within two seconds under the
  // unchanged safety limits. This is not a hardware settling-time requirement.
  EXPECT_LT(final_error, initial_error * .6);
}

INSTANTIATE_TEST_SUITE_P(TranslationAndRotationBothSigns, ClosedLoopSafety, testing::Range(0, 12));
