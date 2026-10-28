// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include "safety_filter/safety_coordinator.hpp"
namespace safety_filter {
struct AsyncSafetyOutput {
  SafetyReply reply;
  bool degraded{false}, entered_degraded{false}, holding{false};
  bool worker_usable{false}, peer_usable{false}, model_reused{false};
  uint64_t worker_age_ns{0};
  std::array<uint64_t, 2> worker_cycles{};
  const char* worker_status{"not_available"};
  const char* reason{"startup"};
};
// One instance per FCI owner. All mutable state is owned by that callback.
// No worker wait and no OSQP call. Fixed model storage and bounded repair work.
class AsyncSafetyController {
 public:
  bool configure(int arm, const CoordinatorParameters& limits, const cbf::SafetyParameters& safety,
                 const std::array<cbf::ArmIdentity, 2>& identities, std::string* error);
  bool prepare_model(cbf::ArmState& state, uint64_t now_ns) noexcept;
  bool model_reused() const noexcept { return model_reused_; }
  AsyncSafetyOutput update(const ModelRequest& request, const ModelRequest* peer,
                           const SafetyReply* result, uint64_t now_ns);

 private:
  cbf::Vector7d braking(const cbf::ArmState& state, double dt, bool reset) noexcept;
  int arm_{0};
  CoordinatorParameters limits_;
  cbf::SafetyParameters safety_;
  cbf::CbfSafetyFilter guard_, solo_guard_;
  SafetyReply cached_;
  bool have_result_{false}, degraded_{false}, have_brake_{false}, holding_{false};
  bool have_model_{false}, model_reused_{false};
  bool peer_was_usable_{false};
  cbf::ArmState last_model_;
  uint64_t model_ns_{0}, healthy_since_{0}, release_before_fault_{0};
  uint64_t pending_epoch_{0}, pending_since_{0};
  cbf::Vector7d brake_q_{cbf::Vector7d::Zero()}, brake_dq_{cbf::Vector7d::Zero()};
};
}  // namespace safety_filter
