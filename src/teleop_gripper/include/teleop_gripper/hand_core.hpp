#pragma once
#include <array>
#include <cstdint>

namespace teleop_gripper {
enum class HandOperation { none, calibrate, close, open, stop };
enum class HandState { unknown, open, stopped, closed, holding };
inline constexpr double kWidthReadbackTolerance = 1e-5;
bool valid_hand_feedback(double width, double maximum_width) noexcept;
// Positive means inward/close for either hand. Joystick deflection selects a
// direction only; speed is fixed and finger travel depends on held duration.
double inward_joystick(double horizontal, bool right_controller) noexcept;
struct HandParameters {
  double engage{0.4}, neutral{0.2}, speed{0.02}, open_speed{0.1};
  double open_width{-1}, close_width{0};
  double cable_diameter{0.005}, cable_tolerance{0.002}, grasp_force{-1};
  uint64_t timeout_ns{100000000}, uncertainty_ns{10000000};
  bool valid() const noexcept;
};
struct GripInput {
  std::array<char, 65> session{};
  uint64_t sequence{0}, source_ns{0}, uncertainty_ns{0}, generation{0};
  double value{0};  // inward-normalized horizontal axis, [-1, 1]
  bool valid{false}, joystick_active{false};
};
struct HandRequest {
  HandOperation operation{HandOperation::none};
  double width{0}, speed{0.02};
  double force{0}, epsilon_inner{0}, epsilon_outer{0};
  double minimum_grasp_width{0}, maximum_grasp_width{0};
};
class HandCore {
 public:
  explicit HandCore(const HandParameters& parameters) : p_(parameters) {}
  HandRequest input(const GripInput& input, uint64_t now_ns) noexcept;
  HandRequest calibrate(bool empty_hand, bool operator_confirm) noexcept;
  HandRequest complete(HandOperation operation, bool success, bool communication_ok, double width,
                       double max_width, uint64_t now_ns, bool grasp_confirmed = false) noexcept;
  HandRequest watchdog(uint64_t now_ns) noexcept;
  HandRequest shutdown() noexcept;
  bool busy() const noexcept { return operation_ != HandOperation::none; }
  bool calibrated() const noexcept { return calibrated_; }
  bool release_required() const noexcept { return need_neutral_; }
  bool grasp_confirmed() const noexcept { return state_ == HandState::holding; }
  double commanded_force() const noexcept { return grasp_confirmed() ? p_.grasp_force : 0.; }
  HandState state() const noexcept { return state_; }
  double width() const noexcept { return width_; }
  double maximum_width() const noexcept { return maximum_width_; }
  uint64_t input_source_ns() const noexcept { return last_.source_ns; }
  const char* message() const noexcept { return message_; }

 private:
  void reset_input() noexcept;
  bool fresh(uint64_t now_ns) const noexcept;
  HandRequest interrupt() noexcept;
  HandRequest start(HandOperation operation) noexcept;
  HandParameters p_;
  GripInput last_{};
  bool identity_{false}, need_neutral_{true}, calibrated_{false}, stopping_{false};
  bool shutdown_{false};
  int direction_{0}, consumed_direction_{0};
  double width_{0}, maximum_width_{0};
  HandOperation operation_{HandOperation::none};
  HandState state_{HandState::unknown};
  const char* message_{"uncalibrated; explicit empty-hand calibration required"};
};
}  // namespace teleop_gripper
