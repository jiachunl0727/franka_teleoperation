#include <gtest/gtest.h>

#include "../include/teleop_controllers/franka_model.hpp"
#include "../src/hardware_startup.hpp"
#include "teleop_controllers/panda_limits.hpp"

TEST(HardwareStartup, IdleWithoutErrorsCanRequestControl) {
  franka::RobotState state{};
  state.robot_mode = franka::RobotMode::kIdle;
  EXPECT_NO_THROW(teleop_controllers::require_idle_control_start(state));
}
TEST(HardwareStartup, ActiveAndFaultedModesNeverRequestCompetingControl) {
  for (auto mode : {franka::RobotMode::kOther, franka::RobotMode::kMove,
                    franka::RobotMode::kGuiding, franka::RobotMode::kReflex,
                    franka::RobotMode::kUserStopped, franka::RobotMode::kAutomaticErrorRecovery}) {
    franka::RobotState state{};
    state.robot_mode = mode;
    EXPECT_THROW(teleop_controllers::require_idle_control_start(state), std::runtime_error);
  }
}
TEST(HardwareStartup, HistoricalErrorsDoNotBlockHealthyIdleStartup) {
  franka::RobotState state{};
  state.robot_mode = franka::RobotMode::kIdle;
  std::array<bool, 41> errors{};
  errors[0] = true;
  state.last_motion_errors = franka::Errors(errors);
  EXPECT_NO_THROW(teleop_controllers::require_idle_control_start(state));
  state.current_errors = state.last_motion_errors;
  EXPECT_THROW(teleop_controllers::require_idle_control_start(state), std::runtime_error);
}

TEST(ModelFeedback, UsesUrdfDynamicsAndSharesOneCoherentModelWithoutRobotAccess) {
  namespace ft = teleop_controllers;
  ft::UrdfPandaModel model(TELEOP_URDF_MODEL_FILE);
  const ft::Vector7 q = Eigen::Map<const ft::Vector7>(ft::kHome.data());
  const ft::Vector7 dq = ft::Vector7::Constant(.1);
  const auto tool = model.standard_flange_to_ee();
  ft::UrdfModelOutput expected;
  ASSERT_TRUE(model.evaluate(q, dq, tool, expected));
  franka::RobotState state{};
  std::copy_n(q.data(), 7, state.q.begin());
  std::copy_n(dq.data(), 7, state.dq.begin());
  std::copy_n(tool.matrix().data(), 16, state.F_T_EE.begin());
  std::copy_n(expected.state.frame_pose[8].matrix().data(), 16, state.O_T_EE.begin());
  state.tau_J_d.fill(.2);
  // Changing Franka's load metadata must not silently change the URDF model.
  state.m_total = 100.;
  state.I_total.fill(10.);
  const auto actual = ft::franka_model_feedback(state, model);
  EXPECT_TRUE(actual.control.mass.isApprox(expected.state.mass, 1e-14));
  EXPECT_TRUE(actual.control.coriolis.isApprox(expected.state.coriolis, 1e-14));
  EXPECT_TRUE(actual.gravity.isApprox(expected.gravity, 1e-14));
  EXPECT_TRUE(actual.model.mass.isApprox(actual.control.mass, 0));
  EXPECT_TRUE(actual.model.coriolis.isApprox(actual.control.coriolis, 0));
  EXPECT_TRUE(actual.control.jacobian.isApprox(actual.model.frame_jacobian[8], 0));
  EXPECT_TRUE(actual.control.previous_torque.isApprox(ft::Vector7::Constant(.2), 0));
}

TEST(HardwareStartup, ObservedTransitionAllowsIdleAndMoveOnly) {
  franka::RobotState state{};
  for (const auto mode : {franka::RobotMode::kIdle, franka::RobotMode::kMove}) {
    state.robot_mode = mode;
    EXPECT_NO_THROW(teleop_controllers::require_startup_observation(state));
  }
  for (const auto mode : {franka::RobotMode::kOther, franka::RobotMode::kGuiding,
                          franka::RobotMode::kReflex, franka::RobotMode::kUserStopped,
                          franka::RobotMode::kAutomaticErrorRecovery}) {
    state.robot_mode = mode;
    EXPECT_THROW(teleop_controllers::require_startup_observation(state), std::runtime_error);
  }
}

TEST(HardwareStartup, ObservedTransitionRejectsCurrentErrorsInEitherMode) {
  franka::RobotState state{};
  std::array<bool,41> errors{}; errors[0]=true;
  state.current_errors=franka::Errors(errors);
  for (const auto mode : {franka::RobotMode::kIdle,franka::RobotMode::kMove}) {
    state.robot_mode=mode;
    EXPECT_THROW(teleop_controllers::require_startup_observation(state),std::runtime_error);
  }
}
