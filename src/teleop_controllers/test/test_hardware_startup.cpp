#include <gtest/gtest.h>

#include "../include/teleop_controllers/franka_model.hpp"
#include "../src/hardware_startup.hpp"

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
