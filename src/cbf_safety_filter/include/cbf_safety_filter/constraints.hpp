#pragma once
#include "cbf_safety_filter/kinematics.hpp"
namespace cbf_safety_filter {
struct ConstraintSet {
  std::array<Row, kMaxRows> rows{};
  int count{0};
  Status status{Status::solved_hard};
  ConstraintFailure failure;
  Vec7 braking_anchor{Vec7::Zero()};
  std::array<double, kMaxRows> requested_bounds{};
  int recovery_rows{0}, outside_domain_rows{0};
};
// Startup-owned workspace. No allocations during build(). Invalid HOCBF
// Out-of-domain rows use the bounded braking recovery rule.
class ConstraintBuilder {
 public:
  explicit ConstraintBuilder(const Configuration& config, bool group_pruning = true);
  const ConstraintSet& build(const ModelView&, double period, uint64_t now_ns,
                             const TorqueBounds* bounds = nullptr,
                             const PeerSnapshot* peer = nullptr,
                             const PeerExchange* exchange = nullptr,
                             bool peer_avoidance = true) noexcept;
  const ConstraintSet& current() const noexcept { return set_; }
  ReferenceNormals reference_normals;
  double model_us{0}, kinematics_us{0}, constraints_us{0};
  // Work counters for numerical/performance regression; no additional timers.
  int group_pruned_pairs{0}, evaluated_peer_pairs{0};

 private:
  bool add(const Vec7&, double, Family, const FamilyConfig&, double h = 1, double psi = 1,
           bool recovery_prepared = false) noexcept;
  bool collision_row(const Vec7&, double, Family, const FamilyConfig&, double, double,
                     const Vec7& gradient, const TorqueBounds*) noexcept;
  Eigen::Matrix<double, 7, 6> task_inverse_{Eigen::Matrix<double, 7, 6>::Zero()};
  Mat7 state_mass_{Mat7::Identity()};
  Configuration config_;
  bool group_pruning_;
  std::vector<std::pair<int, int>> self_pairs_;
  ConstraintSet set_;
  Eigen::LLT<Mat7> mass_;
  std::array<PointKinematics, kMaxPoints> points_{}, bodies_, body_ends_;
  std::array<double, kMaxPoints> body_radii_{};
  bool capsule_row(const PointKinematics&, const PointKinematics&, const PointKinematics&,
                   const PointKinematics&, double radius, Family, const FamilyConfig&,
                   const TorqueBounds*) noexcept;
};
}  // namespace cbf_safety_filter
