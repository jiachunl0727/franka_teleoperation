// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include <atomic>

#include "safety_filter/cbf_safety_filter.hpp"
#include "safety_filter/handoff.hpp"
namespace safety_filter {
enum class SafetyStop : unsigned {
  none,
  result_deadline,
  state_age,
  state_skew,
  invalid_model,
  solver_failures,
  filter_deadline,
  owner_failure,
  scheduling
};
const char* safety_stop_name(SafetyStop reason) noexcept;
struct ModelRequest {
  cbf::ArmState state;
  cbf::Vector7d nominal{cbf::Vector7d::Zero()};
  uint64_t cycle{0}, stamp_ns{0};
  double period{.001};
  bool motion_active{false}, finish_requested{false};
  uint64_t epoch{0}, release_source_ns{0};
};
struct SafetyReply {
  cbf::SafetyResult result;
  std::array<uint64_t, 2> cycles{}, source_ns{};
  uint64_t started_ns{0}, completed_ns{0};
  unsigned consecutive_failures{0};
  bool finish{false};
  cbf::Vector14d nominal{cbf::Vector14d::Zero()};
  std::array<cbf::Vector7d, 2> source_q{cbf::Vector7d::Zero(), cbf::Vector7d::Zero()},
      source_dq{cbf::Vector7d::Zero(), cbf::Vector7d::Zero()};
  std::array<uint64_t, 2> epoch{};
};
struct CoordinatorParameters {
  std::array<bool, 2> active{true, false};
  uint64_t maximum_age_ns{2000000}, maximum_skew_ns{1000000};
  uint64_t result_deadline_ns{900000}, filter_deadline_ns{800000};
  unsigned maximum_failures{20};
  bool asynchronous{false};  // Runtime selects true; synchronous API retained for comparison.
  uint64_t worker_period_ns{5000000}, maximum_result_age_ns{50000000};
  uint64_t maximum_worker_state_age_ns{20000000}, maximum_peer_age_ns{10000000};
  uint64_t recovery_dwell_ns{100000000}, model_grace_ns{3000000};
  double result_joint_change{0.10}, result_velocity_change{1.0};
  double braking_acceleration{2.0}, hold_stiffness{120.0}, hold_damping{22.0};
  double clearance_reserve{0.005}, peer_motion_bound{22.0};
  bool valid() const noexcept;
};
// One producer/consumer per arm lane. Default runtime callbacks publish latest
// requests and consume available results without waiting; the worker remains
// coupled in dual mode. The explicit synchronous comparison API retains exact
// cycle/deadline checks. Shared stop and finish arbitration remain. No ROS/mutexes.
class SafetyCoordinator {
 public:
  bool configure(const CoordinatorParameters& limits, const cbf::SafetyParameters& safety,
                 const std::array<cbf::ArmIdentity, 2>& identities, std::string* error);
  void submit(int arm, const ModelRequest& request) noexcept {
    requests_[arm].publish(request);
    peer_requests_[1 - arm].publish(request);
    finishing_[arm].store(request.finish_requested, std::memory_order_release);
  }
  bool peer(int arm, ModelRequest& request) noexcept { return peer_requests_[arm].read(request); }
  bool may_finish() const noexcept {
    for (int i = 0; i < 2; ++i)
      if (limits_.active[i] && !finishing_[i].load(std::memory_order_acquire))
        return false;
    return true;
  }
  bool receive(int arm, SafetyReply& reply) noexcept { return replies_[arm].read(reply); }
  bool pump(uint64_t now_ns = 0);
  bool accepts(int arm, uint64_t cycle, uint64_t started_ns, uint64_t now_ns,
               const SafetyReply& reply) noexcept;
  void stop(SafetyStop reason) noexcept;
  SafetyStop stopped() const noexcept { return stop_.load(std::memory_order_acquire); }
  const CoordinatorParameters& limits() const noexcept { return limits_; }
  // Postmortem access only: the caller must join the pump thread first.
  const ModelRequest& diagnostic_request(int arm) const { return pending_.at(arm); }
  const char* diagnostic_filter_status() const noexcept { return filter_status_; }
  const SafetyReply& diagnostic_reply() const noexcept { return diagnostic_reply_; }
  const SafetyReply& diagnostic_reply(int arm) const { return lane_diagnostics_.at(arm); }

 private:
  bool process(const std::array<bool, 2>& active, cbf::CbfSafetyFilter& filter, unsigned& failures,
               uint64_t now_ns);
  cbf::CbfSafetyFilter filter_;
  std::array<cbf::CbfSafetyFilter, 2> actuator_filters_;
  bool independent_actuators_{false};
  CoordinatorParameters limits_;
  std::array<LatestSnapshot<ModelRequest>, 2> requests_;
  std::array<LatestSnapshot<ModelRequest>, 2> peer_requests_;
  std::array<std::atomic<bool>, 2> finishing_{};
  std::array<LatestSnapshot<SafetyReply>, 2> replies_;
  std::array<ModelRequest, 2> pending_;
  std::array<bool, 2> new_{};
  unsigned failures_{0};
  std::array<unsigned, 2> lane_failures_{};
  const char* filter_status_{"not_run"};
  SafetyReply diagnostic_reply_;
  std::array<SafetyReply, 2> lane_diagnostics_;
  std::atomic<SafetyStop> stop_{SafetyStop::none};
  uint64_t last_worker_ns_{0};
  std::array<uint64_t, 2> last_source_ns_{};
};
}  // namespace safety_filter
