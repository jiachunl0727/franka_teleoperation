#pragma once

#include <franka/robot_state.h>

#include <stdexcept>

#include "teleop_controllers/control_types.hpp"
#include "teleop_controllers/legacy_pose.hpp"
#include "teleop_controllers/robot_model.hpp"
#include "teleop_controllers/urdf_model.hpp"

namespace teleop_controllers {
struct FrankaModelFeedback {
  ArmState control;
  RobotModelState model;
  Vector7 gravity{Vector7::Zero()};
};
// Adapt one RobotState coherently. No connection, control, caching or scheduling.
// previous_torque is tau_J_d under the gravity-compensated command convention.
inline FrankaModelFeedback franka_model_feedback(const franka::RobotState& state,
                                                 const UrdfPandaModel& model) {
  FrankaModelFeedback out;
  out.control.q = Eigen::Map<const Vector7>(state.q.data());
  out.control.dq = Eigen::Map<const Vector7>(state.dq.data());
  out.control.previous_torque = Eigen::Map<const Vector7>(state.tau_J_d.data());
  Eigen::Affine3d pose(Eigen::Map<const Eigen::Matrix4d>(state.O_T_EE.data()));
  if (!teleop_controllers::orthonormalize_legacy_pose(pose))
    throw std::runtime_error("invalid measured end-effector transform");
  out.control.position = pose.translation();
  out.control.orientation = Eigen::Quaterniond(pose.linear());
  Eigen::Affine3d tool(Eigen::Map<const Eigen::Matrix4d>(state.F_T_EE.data()));
  if (!orthonormalize_legacy_pose(tool))
    throw std::runtime_error("invalid flange-to-end-effector transform");
  UrdfModelOutput evaluated;
  if (!model.evaluate(out.control.q, out.control.dq, tool, evaluated))
    throw std::runtime_error("invalid URDF model state");
  out.model = evaluated.state;
  out.control.jacobian = out.model.frame_jacobian[8];
  out.control.coriolis = out.model.coriolis;
  out.control.mass = out.model.mass;
  out.gravity = evaluated.gravity;
  out.model.previous_torque = out.control.previous_torque;
  return out;
}
}  // namespace teleop_controllers
