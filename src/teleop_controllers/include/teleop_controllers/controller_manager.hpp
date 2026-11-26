#pragma once

#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/homing_controller.hpp"

namespace teleop_controllers {
struct ManagerParameters {
  uint64_t timeout_ns{100000000}, uncertainty_ns{10000000}, future_ns{10000000};
  double torque_rate{teleop_controllers::panda_torque_rate_limit()};
  Vector7 torque_limits{teleop_controllers::panda_torque_limits()};
  // Controller reference and tracking-response settings.
  bool reference_shaping_enabled{false}, tracking_protection_enabled{false};
  double reference_linear_speed{0.05}, reference_angular_speed{0.15};
  double reference_linear_acceleration{0.15}, reference_angular_acceleration{0.5};
  double tracking_position_limit{0.03}, tracking_orientation_limit{0.20};
  bool valid() const noexcept;
};

struct ManagerOutput {
  Output control;
  HomingOutput homing;
};

// One per arm, called only by its permanent control thread. Owns input gates,
// mode transitions and selection of exactly one nominal torque law per cycle.
// Laws are constructed before realtime control and must outlive the manager.
class ControllerManager {
 public:
  ControllerManager(const ManagerParameters& parameters, const HomingParameters& homing_parameters,
                    TorqueController& teleoperation, TorqueController& orientation_only,
                    TorqueController& position_hold, TorqueController& damping)
      : p_(parameters),
        homing_(homing_parameters),
        teleoperation_(teleoperation),
        orientation_only_(orientation_only),
        position_hold_(position_hold),
        damping_(damping) {}
  ControllerManager(const ManagerParameters& parameters, const HomingParameters& homing_parameters,
                    TorqueController& teleoperation, TorqueController& position_hold,
                    TorqueController& damping)
      : ControllerManager(parameters, homing_parameters, teleoperation, teleoperation,
                          position_hold, damping) {}
  void accept(const Command& command, uint64_t now_ns) noexcept;
  // Optional owner-supplied reference governor. No numerical safety dependency.
  using ReferenceProjection = Vector6 (*)(const void*, const Vector6&) noexcept;
  void reference_projection(ReferenceProjection function, const void* context,
                            bool discard_blocked_tracking_error = false) noexcept {
    reference_projection_ = function;
    projection_context_ = context;
    discard_blocked_tracking_error_ = discard_blocked_tracking_error;
  }
  const Command& requested_command() const noexcept { return last_; }
  uint64_t release_source_ns() const noexcept { return last_release_source_ns_; }
  void homing_input(const HomingInput& input, uint64_t now_ns) noexcept {
    if (!shutdown_ && !protective_stop_)
      homing_.input(input, now_ns);
  }
  void cancel_homing(const char* reason) noexcept { homing_.cancel(reason); }
  void fault(Reason reason) noexcept;
  void request_protective_stop(Reason reason) noexcept { protect(reason); }
  void shutdown() noexcept;
  // Called once after mode selection, before governing an active reference.
  // Lets the owner prepare fresh constraints for the selected motion/hold law.
  using BeforeReference = void (*)(void*, bool movable) noexcept;
  ManagerOutput update(const ArmState& state, uint64_t now_ns, double period,
                       BeforeReference prepare = nullptr, void* context = nullptr) noexcept;

 private:
  bool fresh(const Command& command, uint64_t now_ns) const noexcept;
  void protect(Reason reason) noexcept;
  CartesianReference govern(const ArmState& state, const CartesianReference& goal, double dt,
                            bool reset) noexcept;
  Vector7 protective_torque(const ArmState& state) noexcept;
  ReferenceProjection reference_projection_{nullptr};
  bool discard_blocked_tracking_error_{false};
  const void* projection_context_{nullptr};
  CartesianReference safety_reference_{};
  CartesianReference projection_input_{}, projection_goal_{};
  ManagerParameters p_;
  CartesianReference governed_{};
  bool protective_stop_{false}, protective_holding_{false};
  Reason protective_reason_{Reason::startup};
  Vector7 protective_q_{Vector7::Zero()};
  double blocked_seconds_{0};
  HomingController homing_;
  TorqueController& teleoperation_;
  TorqueController& orientation_only_;
  TorqueController& position_hold_;
  TorqueController& damping_;
  TorqueController* selected_{nullptr};
  JointReference hold_reference_;
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_hold_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond hold_orientation_{1, 0, 0, 0};
  Command last_{};
  std::array<char, 65> session_{};
  uint64_t generation_{0}, sequence_{0}, source_ns_{0};
  uint64_t last_release_source_ns_{0};
  bool identity_{false}, ordered_{false}, release_seen_{false}, enabled_{false};
  bool shutdown_{false}, teleoperation_reset_pending_{true};
  bool hold_reset_pending_{true};
  bool hold_permitted_{true};
  // Local position holding is independent of Quest readiness at startup and
  // between engagements. Input faults revoke arming, never an idle hold anchor.
  bool local_hold_{true};
  Reason reason_{Reason::startup};
};
}  // namespace teleop_controllers
