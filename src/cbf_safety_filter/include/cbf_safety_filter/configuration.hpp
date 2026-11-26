#pragma once
#include <algorithm>
#include <string>
#include <vector>

#include "cbf_safety_filter/types.hpp"
namespace cbf_safety_filter {
struct FamilyConfig {
  bool enabled{false};
  double margin{.1}, alpha1{5}, alpha2{5}, height{0}, clearance{.05};
};
struct PointDefinition {
  int frame{7};
  Eigen::Vector3d offset{Eigen::Vector3d::Zero()};
  double radius{.04};
};
struct Capsule {
  int frame{7};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()}, b{Eigen::Vector3d::Zero()};
  double radius{.04};
};
struct SphereObstacle {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{.1};
};
struct CollisionPair {
  int point{0}, obstacle{0};
};
struct BodySphere : PointDefinition {
  int group{0};  // Source rigid link, including fixed base (-1 frame).
  Eigen::Vector3d span{Eigen::Vector3d::Zero()};
  Eigen::Vector3d opening_axis{Eigen::Vector3d::Zero()};  // Unit axis for the sliding finger.
  bool used_for_capsule_table() const noexcept { return group >= 2; }
};
struct SelfExclusion {
  int first{0}, second{0};
};
enum class RecoveryTorqueMethod { projection, native_qp };
struct Configuration {
  Backend backend{Backend::osqp};
  int max_constraints{256};
  double max_filter_time_us{600}, max_model_age_us{2000}, max_callback_time_us{800};
  Vec7 weights{Vec7::Ones()},
      acceleration_limits{(Vec7() << 15, 7.5, 10, 12.5, 15, 20, 20).finished()};
  Vec7 position_lower_limits{
      (Vec7() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -.0175, -2.8973).finished()},
      position_upper_limits{
          (Vec7() << 2.8973, 1.7628, 2.8973, -.0698, 2.8973, 3.7525, 2.8973).finished()},
      velocity_limits{(Vec7() << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61).finished()};
  bool warm_start{true}, analytic_nominal{true};
  int osqp_iterations{100}, qpoases_recalculations{100}, piqp_iterations{50};
  double osqp_abs{1e-5}, osqp_rel{1e-5}, qpoases_tolerance{1e-6}, piqp_abs{1e-6}, piqp_rel{1e-6};
  double hard_tolerance{1e-6};
  FamilyConfig position, velocity, acceleration, table, obstacles;
  FamilyConfig self_collision, inter_arm, singularity;
  double singularity_length{0.3}, singularity_minimum{0.025};
  double boundary_activation{0.03};
  // Zero inherits the common band; an explicit value affects inter-arm only.
  double inter_arm_boundary_activation{0};
  double inter_arm_activation() const noexcept {
    return inter_arm_boundary_activation > 0 ? inter_arm_boundary_activation : boundary_activation;
  }
  double peer_max_age_us{3000}, peer_max_skew_us{3000};
  bool uses_body_volumes{false};
  RecoveryTorqueMethod recovery_torque_method{RecoveryTorqueMethod::projection};
  double recovery_braking_gain{20};
  int recovery_projection_sweeps{6};
  double point_speed_limit{1.0}, point_acceleration_limit{5.0};
  std::vector<BodySphere> body_spheres;
  std::vector<SelfExclusion> self_exclusions;
  Eigen::Affine3d world_from_base{Eigen::Affine3d::Identity()};
  std::vector<Capsule> capsules;
  std::vector<PointDefinition> points;
  std::vector<SphereObstacle> obstacle_spheres;
  std::vector<CollisionPair> pairs;
  bool diagnostic_residuals{true}, diagnostic_times{true};
  void validate() const;
};
// A shared dual profile selects geometry by the owner's Quest controller ID.
// Single-geometry profiles remain usable without a selector.
Configuration load_configuration(const std::string& path, const std::string& controller_id = "");
}  // namespace cbf_safety_filter
