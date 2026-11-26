#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace cbf_safety_filter {
constexpr int kJoints = 7, kFrames = 9, kMaxRows = 512, kMaxPoints = 64;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat7 = Eigen::Matrix<double, 7, 7>;
using Jacobian = Eigen::Matrix<double, 6, 7>;
using PointJacobian = Eigen::Matrix<double, 3, 7>;
enum class Backend { osqp, qpoases, piqp };
enum class Family {
  position,
  velocity,
  acceleration,
  table,
  obstacle,
  self_collision,
  inter_arm,
  singularity,
  peer_motion
};
enum class Status {
  disabled,
  solved_hard,
  infeasible,
  iteration_limit,
  numerical_failure,
  invalid_model,
  invalid_domain,
  constraint_violation,
  capacity_exceeded,
  timing_overrun,
  stale_peer,
  verified_recovery
};
const char* name(Status) noexcept;
const char* name(Backend) noexcept;
// Borrowed coherent BASE-frame data; references must outlive compute().
// Spatial Jacobians are [linear;angular] in BASE, at frame origin.
struct FingerOpening {
  double width{0};
  uint64_t sample_ns{0};
};
struct ModelView {
  const Vec7& q;
  const Vec7& dq;
  const Mat7& mass;
  const Vec7& coriolis;
  const std::array<Eigen::Affine3d, kFrames>& poses;
  const std::array<Jacobian, kFrames>& jacobians;
  uint64_t timestamp_ns;
  FingerOpening finger_opening{};
};
struct WorldSphere {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
  double radius{0};
  int group{0};                                   // Performance grouping only.
  Eigen::Vector3d span{Eigen::Vector3d::Zero()};  // Capsule endpoint minus start.
};
// Value snapshot: one writer/reader per link; no torque candidates are exchanged.
struct PeerSnapshot {
  std::array<WorldSphere, kMaxPoints> spheres{};
  int count{0};
  uint64_t timestamp_ns{0};
  double speed_limit{0}, acceleration_limit{0};
  bool valid{false};
};
// Optional same-owner, bounded in-process handoff. Invoked after local geometry
// is evaluated, so publication and local constraints share exactly those values.
// No waits, network/ROS operations, model queries or heap work in this hook.
struct PeerExchange {
  void* context{nullptr};
  const PeerSnapshot* (*publish_read)(void*, const PeerSnapshot&) noexcept {nullptr};
  // One bounded refresh immediately before peer validation, after local rows.
  const PeerSnapshot* (*read_latest)(void*) noexcept {nullptr};
};
using Vec6 = Eigen::Matrix<double, 6, 1>;
struct ReferenceNormals {
  std::array<Vec6, kMaxRows> normals{};
  int count{0};
  // Supervisory projection only. Fresh torque inequalities remain authoritative.
  Vec6 project(Vec6 step) const noexcept;
};
struct TorqueBounds {
  Vec7 lower, upper;
};
struct Row {
  Vec7 g{Vec7::Zero()};
  double bound{0};
  Family family{Family::position};
  double h{0}, psi{0};
  Vec6 reference_normal{Vec6::Zero()};
};
// The first rejected row is not inserted into the valid constraint set.
// Retain it separately so early prepare() failures can be diagnosed.
struct ConstraintFailure {
  bool present{false};
  int row_index{-1};
  Row row;
};
struct Result {
  Status status{Status::numerical_failure};
  Backend backend{Backend::osqp};
  bool analytic_solution{false};
  Vec7 torque{Vec7::Zero()};
  int rows{0}, iterations{0};
  int group_pruned_pairs{0}, evaluated_peer_pairs{0};
  int recovery_rows{0}, outside_domain_rows{0};
  double max_requested_violation{0};
  Status requested_qp_status{Status::disabled}, recovery_qp_status{Status::disabled};
  bool recovery_projection_fallback{false};
  double max_hard_violation{0}, intervention_norm{0};
  double model_us{0}, kinematics_us{0}, constraints_us{0}, assembly_us{0}, update_us{0};
  double solve_us{0}, verify_us{0}, total_us{0};
  std::array<double, kMaxRows> original_residual{};
  std::array<Family, kMaxRows> families{};
  bool accepted() const noexcept {
    return status == Status::solved_hard || status == Status::verified_recovery;
  }
};
}  // namespace cbf_safety_filter
