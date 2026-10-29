// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
// Fixed model snapshots adapted from thesis_ws's dual-arm control types.
#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>
namespace teleop_controllers {
constexpr int kJointsPerArm = 7, kModelFrames = 9;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using ModelJacobian = Eigen::Matrix<double, 6, 7>;
struct RobotModelState {
  Vector7d q{Vector7d::Zero()}, dq{Vector7d::Zero()}, coriolis{Vector7d::Zero()};
  Vector7d previous_torque{Vector7d::Zero()};
  Matrix7d mass{Matrix7d::Identity()};
  std::array<Eigen::Affine3d, kModelFrames> frame_pose{};
  std::array<ModelJacobian, kModelFrames> frame_jacobian{};
  RobotModelState() {
    for (auto& pose : frame_pose)
      pose.setIdentity();
    for (auto& j : frame_jacobian)
      j.setZero();
  }
};
}  // namespace teleop_controllers
