#pragma once

#include <ostream>
#include <string>

#include "cbf_safety_filter/safety_filter.hpp"
#include "teleop_controllers/robot_model.hpp"

namespace teleop_controllers {
struct FciTiming {
  bool valid{false}, waited{false};
  uint64_t state_id{0};
  uint32_t received_states{0};
  double update_wall_us{0}, update_cpu_us{0}, receive_wall_us{0}, post_receive_us{0},
      to_callback_us{0};
};
void write_fci_timing(std::ostream&, const FciTiming&);
// Owner-thread value snapshot captured once when a CBF failure policy is applied.
// Formatting/publication happens after the callback and robot.control exit.
struct CbfTerminalSnapshot {
  FciTiming fci;
  bool present{false}, nominal_available{false}, filter_timing_valid{false},
      previous_command_valid{false}, requested_pose_valid{false},
      protective_command_issued{false}, control_terminated{false};
  const char* stage{"prepare"};
  cbf_safety_filter::Status status{cbf_safety_filter::Status::disabled};
  cbf_safety_filter::ConstraintFailure failure;
  cbf_safety_filter::Result result;
  cbf_safety_filter::TorqueBounds bounds;
  cbf_safety_filter::PeerSnapshot peer;
  RobotModelState model;
  Vector7d nominal{Vector7d::Zero()}, last_command{Vector7d::Zero()},
      protective_command{Vector7d::Zero()};
  Vector7d measured_torque{Vector7d::Zero()}, gravity{Vector7d::Zero()};
  Eigen::Vector3d requested_position{Eigen::Vector3d::Zero()},
      reference_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond requested_orientation{1, 0, 0, 0}, reference_orientation{1, 0, 0, 0};
  uint64_t cycle{0}, state_ns{0}, previous_cycle{0}, previous_issue_ns{0};
  uint64_t control_request_ns{0}, own_startup_seed_ns{0}, peer_startup_seed_ns{0};
  uint64_t startup_observation_count{0}, last_startup_observation_ns{0};
  double period{0}, observed_period_us{0}, callback_us{0}, callback_cpu_us{0},
      model_acquisition_us{0};
  int callback_cpu{-1};
};
// JSON single line suitable for a tee'd launch log; nonfinite values become null.
void write_cbf_terminal_snapshot(std::ostream&, const CbfTerminalSnapshot&,
                                 const std::string& model_fingerprint);
}  // namespace teleop_controllers
