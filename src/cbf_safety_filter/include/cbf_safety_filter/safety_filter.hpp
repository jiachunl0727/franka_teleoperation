#pragma once
#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf_safety_filter {
// Observation only: geometric/actuator feasibility is not command authority.
// Timing, proposal freshness and transaction authorization remain separate.
struct CommandInspection {
  bool domain_valid{false}, finite{false}, actuator_feasible{false}, rows_feasible{false};
  double max_actuator_violation{0}, max_row_violation{0};
  double max_box_min_row_violation{0};
  int worst_family{-1};
  int impossible_rows{0}, impossible_family{-1};
  bool feasible() const noexcept {
    return domain_valid && finite && actuator_feasible && rows_feasible;
  }
};
class SafetyFilter {
 public:
  explicit SafetyFilter(const Configuration& config);
  const Result& compute(const ModelView&, const Vec7& nominal, const TorqueBounds&, double period,
                        uint64_t now_ns, const PeerSnapshot* peer = nullptr) noexcept;
  const ReferenceNormals& reference_normals() const noexcept { return reference_normals_; }
  const ConstraintFailure& constraint_failure() const noexcept {
    return builder_.current().failure;
  }
  // Single-use transaction: prepare fresh rows, govern/reference + nominal, then solve.
  Status prepare(const ModelView&, const TorqueBounds&, double period, uint64_t now_ns,
                 const PeerSnapshot* peer = nullptr, const PeerExchange* exchange = nullptr,
                 bool peer_avoidance = true) noexcept;
  const Result& solve_prepared(const Vec7& nominal) noexcept;
  // Recheck the exact outgoing torque against this cycle's current rows.
  const Result& last_result() const noexcept { return result_; }
  bool verify_command(const Vec7&) const noexcept;
  // A held arm may retain its pose while its OWN safety limits shape torque.
  // This query must certify that inter-arm rows did not cause the correction.
  // Fail closed if the current solution's local-only provenance is unproven.
  // All original inter-arm inequalities remain in the QP and outgoing check.
  bool hold_correction_is_local(const Vec7& nominal) const noexcept;
  // Inspect any torque against this cycle's original rows. This never
  // authorizes torque or changes the filter result.
  CommandInspection inspect_command(const Vec7&) const noexcept;
  const Configuration& configuration() const noexcept { return config_; }
  const ConstraintSet& constraints() const noexcept { return builder_.current(); }

 private:
  bool checked_candidate(const Vec7&, const Vec7&, Status) noexcept;
  void finish() noexcept;
  Result solve_qp(const Vec7& nominal, const double* bound_override = nullptr) noexcept;
  void solve_recovery_qp(const Vec7& nominal) noexcept;
  Configuration config_;
  ReferenceNormals reference_normals_, previous_binding_;
  uint64_t source_ns_{0};
  bool prepared_{false}, command_ready_{false};
  uint64_t preparation_ns_{0};
  double preparation_age_us_{0};
  ConstraintBuilder builder_;
  QpProblem problem_;
  std::unique_ptr<QpSolver> solver_;
  Result result_;
  TorqueBounds bounds_;
};
}  // namespace cbf_safety_filter
