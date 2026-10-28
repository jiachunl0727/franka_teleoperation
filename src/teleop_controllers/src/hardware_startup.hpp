#pragma once

#include <franka/robot_state.h>

#include <sstream>
#include <stdexcept>

namespace teleop_controllers {
inline void require_idle_control_start(const franka::RobotState& state) {
  if (state.robot_mode == franka::RobotMode::kIdle && !state.current_errors)
    return;
  std::ostringstream error;
  error << "FCI torque control requires Idle with no current errors; robot reports "
        << state.robot_mode << ", current_errors=" << state.current_errors
        << ". Finish the active motion/control session and recover errors in Desk before "
           "relaunching. No automatic recovery or competing control request was issued.";
  throw std::runtime_error(error.str());
}
}  // namespace teleop_controllers
