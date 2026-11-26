#pragma once
#include "cbf_safety_filter/configuration.hpp"
namespace cbf_safety_filter {
struct PointKinematics {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_bias{Eigen::Vector3d::Zero()};  // Jdot*qdot, qdd=0
  PointJacobian jacobian{PointJacobian::Zero()}, torque_map{PointJacobian::Zero()};
  Eigen::Vector3d drift_acceleration{Eigen::Vector3d::Zero()};
};
// Revolute joint axes/origins from the coherent model; no FK/model queries or
// cross-cycle numerical differentiation. All outputs are in WORLD.
PointKinematics point_kinematics(const ModelView&, const PointDefinition&,
                                 const Eigen::Affine3d& world_from_base) noexcept;
// Callback-local frame derivatives, shared by all offsets on a rigid link.
// Owns values, never retains a borrowed ModelView or a cross-cycle cache.
class PointKinematicsWorkspace {
 public:
  PointKinematicsWorkspace(const ModelView&, const Eigen::Affine3d&) noexcept;
  PointKinematics evaluate(const PointDefinition&) const noexcept;

 private:
  Vec7 dq_;
  std::array<Eigen::Affine3d, kFrames> poses_;
  std::array<Jacobian, kFrames> jacobians_;
  std::array<Eigen::Vector3d, kFrames> omega_, angular_bias_, origin_bias_;
};
struct SingularityKinematics {
  double value{0}, bias{0};
  Vec7 gradient{Vec7::Zero()};
  bool valid{false};
};
// 1 / trace((J_normalized J_normalized^T)^-1), a smooth lower bound
// on sigma_min^2 in the full-rank domain. Analytic joint/directional derivatives.
SingularityKinematics singularity_kinematics(const ModelView&, double length) noexcept;
BodySphere position_body_volume(const BodySphere&, const ModelView&, const Configuration&) noexcept;
PeerSnapshot collision_snapshot(const ModelView&, const Configuration&) noexcept;
}  // namespace cbf_safety_filter
