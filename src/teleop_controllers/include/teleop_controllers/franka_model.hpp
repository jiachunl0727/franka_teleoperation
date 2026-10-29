#pragma once

#include <franka/model.h>
#include <franka/robot_state.h>

#include <stdexcept>

#include "teleop_controllers/control_types.hpp"
#include "teleop_controllers/legacy_pose.hpp"
#include "teleop_controllers/robot_model.hpp"

namespace teleop_controllers {
struct FrankaModelFeedback {
  ArmState control;
  RobotModelState model;
  Vector7 gravity{Vector7::Zero()};
};
// Adapt one RobotState coherently. No connection, control, caching or scheduling.
// previous_torque is tau_J_d under the gravity-compensated command convention.
inline FrankaModelFeedback franka_model_feedback(const franka::RobotState& state,
                                                 const franka::Model& model) {
  FrankaModelFeedback out;
  out.control.q = Eigen::Map<const Vector7>(state.q.data());
  out.control.dq = Eigen::Map<const Vector7>(state.dq.data());
  out.control.previous_torque = Eigen::Map<const Vector7>(state.tau_J_d.data());
  Eigen::Affine3d pose(Eigen::Map<const Eigen::Matrix4d>(state.O_T_EE.data()));
  if (!teleop_controllers::orthonormalize_legacy_pose(pose))
    throw std::runtime_error("invalid measured end-effector transform");
  out.control.position = pose.translation();
  out.control.orientation = Eigen::Quaterniond(pose.linear());
  const auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
  const auto coriolis = model.coriolis(state);
  const auto gravity = model.gravity(state);
  const auto mass = model.mass(state);
  out.control.jacobian = Eigen::Map<const Jacobian>(jacobian.data());
  out.control.coriolis = Eigen::Map<const Vector7>(coriolis.data());
  out.control.mass = Eigen::Map<const teleop_controllers::Matrix7d>(mass.data());
  out.gravity = Eigen::Map<const Vector7>(gravity.data());
  out.model.q = out.control.q;
  out.model.dq = out.control.dq;
  out.model.coriolis = out.control.coriolis;
  out.model.mass = out.control.mass;
  out.model.previous_torque = out.control.previous_torque;
  for (int index = 0; index < teleop_controllers::kModelFrames; ++index) {
    const auto frame = static_cast<franka::Frame>(index);
    const auto transform = model.pose(frame, state);
    const auto j = model.zeroJacobian(frame, state);
    out.model.frame_pose[index] =
        Eigen::Affine3d(Eigen::Map<const Eigen::Matrix4d>(transform.data()));
    if (!teleop_controllers::orthonormalize_legacy_pose(out.model.frame_pose[index]))
      throw std::runtime_error("invalid model frame transform");
    out.model.frame_jacobian[index] = Eigen::Map<const Jacobian>(j.data());
  }
  return out;
}
}  // namespace teleop_controllers
