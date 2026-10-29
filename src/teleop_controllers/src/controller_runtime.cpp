#include <franka/control_types.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <sys/mman.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <franka_msgs/msg/franka_state.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <quest_teleop_interfaces/msg/cartesian_teleop_command.hpp>
#include <quest_teleop_interfaces/msg/controller_status.hpp>
#include <quest_teleop_interfaces/msg/homing_status.hpp>
#include <quest_teleop_interfaces/msg/quest_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "hardware_startup.hpp"
#include "teleop_controllers/cartesian_impedance_controller.hpp"
#include "teleop_controllers/controller_manager.hpp"
#include "teleop_controllers/damping_controller.hpp"
#include "teleop_controllers/franka_model.hpp"
#include "teleop_controllers/handoff.hpp"
#include "teleop_controllers/model_snapshot.hpp"
#include "teleop_controllers/orientation_only_controller.hpp"
#include "teleop_controllers/panda_model_fixture.hpp"
#include "teleop_controllers/position_hold_controller.hpp"
#include "tracking_message.hpp"

namespace ft = teleop_controllers;
namespace local = teleop_controllers;
using Clock = std::chrono::steady_clock;
uint64_t monotonic_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
}
struct Telemetry {
  ft::ArmState state;
  ft::Output output;
  ft::HomingOutput homing;
  franka::RobotState native;
  local::RobotModelState model;
  ft::Vector7 gravity{ft::Vector7::Zero()};
  bool finish_requested{false};
  bool motion_requested{false};
  uint64_t stamp_ns{0}, cycle{0};
};
struct PoseFeedback {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{1, 0, 0, 0};
  uint64_t stamp_ns{0}, cycle{0};
};
// Process lifecycle only. There is no safety worker, candidate cache or
// safety-mode arbitration in this runtime while its replacement is unimplemented.
struct RuntimeSession {
  std::atomic<bool> failed{false};
  std::array<std::atomic<bool>, 2> finishing{};
  int arms{1};
  bool may_finish() const noexcept {
    for (int i = 0; i < arms; ++i)
      if (!finishing[i].load(std::memory_order_acquire))
        return false;
    return true;
  }
};
struct MotionDiagnostic {
  local::ModelSnapshot model;
  uint64_t stamp_ns{0};
  ft::Reason reason{ft::Reason::startup};
  ft::Command command;
  ft::Vector7 measured_torque{ft::Vector7::Zero()}, gravity{ft::Vector7::Zero()};
};
void save_model_diagnostic(const rclcpp::Logger& logger, const local::ModelSnapshot& snapshot,
                           const std::string& path);
void write_pose(geometry_msgs::msg::Pose& p, const Eigen::Vector3d& v,
                const Eigen::Quaterniond& q) {
  p.position.x = v.x();
  p.position.y = v.y();
  p.position.z = v.z();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
}
template <class T>
T parameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, const T& value) {
  if (node->has_parameter(name))
    return node->get_parameter(name).get_value<T>();
  return node->declare_parameter<T>(name, value);
}
int run_arm(const std::shared_ptr<rclcpp::Node>& node, int arm, RuntimeSession& session,
            std::atomic<unsigned>& prepared, std::atomic<unsigned>& connected, unsigned count) {
  int result = 0;
  bool owner_fault = false;
  double fault_period = 0;
  Telemetry fault_sample;
  try {
    const auto backend = parameter<std::string>(node, "backend", "mock");
    const auto ip = parameter<std::string>(node, "robot_ip", "");
    const auto confirmed = parameter<bool>(node, "operator_confirm", false);
    const auto frame = parameter<std::string>(node, "base_frame", "panda_1/panda_link0");
    const auto command_topic =
        parameter<std::string>(node, "command_topic", "cartesian_teleop_command");
    const auto pose_topic =
        parameter<std::string>(node, "measured_pose_topic", "teleop_anchor_pose");
    const auto state_topic = parameter<std::string>(node, "state_topic", "robot_state");
    const auto controller_id = parameter<std::string>(node, "controller_id", "right");
    const auto raw_topic = parameter<std::string>(
        node, "controller_state_topic", "/quest_usb_bridge/" + controller_id + "/controller_state");
    const auto home_button = controller_id == "left" ? "x" : "a";
    const double trigger_release = parameter(node, "trigger_release", .25);
    ft::ManagerParameters p;
    ft::CartesianImpedanceParameters impedance;
    impedance.lambda_translation =
        parameter(node, "lambda_translation", impedance.lambda_translation);
    impedance.lambda_rotation = parameter(node, "lambda_rotation", impedance.lambda_rotation);
    impedance.gain_translation = parameter(node, "gain_translation", impedance.gain_translation);
    impedance.gain_rotation = parameter(node, "gain_rotation", impedance.gain_rotation);
    impedance.nullspace_enabled = parameter(node, "nullspace_enabled", true);
    const auto home = parameter<std::vector<double>>(
        node, "home_joint_position", std::vector<double>(local::kHome.begin(), local::kHome.end()));
    if (home.size() != 7)
      throw std::invalid_argument("home_joint_position requires seven values");
    impedance.home_joint_position = Eigen::Map<const ft::Vector7>(home.data());
    impedance.nullspace_stiffness =
        parameter(node, "nullspace_stiffness", impedance.nullspace_stiffness);
    impedance.nullspace_damping = parameter(node, "nullspace_damping", impedance.nullspace_damping);
    impedance.nullspace_joint_limit_gain =
        parameter(node, "nullspace_joint_limit_gain", impedance.nullspace_joint_limit_gain);
    impedance.nullspace_joint_limit_margin =
        parameter(node, "nullspace_joint_limit_margin", impedance.nullspace_joint_limit_margin);
    impedance.nullspace_max_torque =
        parameter(node, "nullspace_max_torque", impedance.nullspace_max_torque);
    impedance.nullspace_ramp_seconds =
        parameter(node, "nullspace_ramp_seconds", impedance.nullspace_ramp_seconds);
    p.reference_linear_speed = parameter(node, "reference_linear_speed", p.reference_linear_speed);
    p.reference_shaping_enabled =
        parameter(node, "reference_shaping_enabled", p.reference_shaping_enabled);
    p.tracking_protection_enabled =
        parameter(node, "tracking_protection_enabled", p.tracking_protection_enabled);
    impedance.cartesian_wrench_limits_enabled = parameter(
        node, "cartesian_wrench_limits_enabled", impedance.cartesian_wrench_limits_enabled);
    p.reference_angular_speed =
        parameter(node, "reference_angular_speed", p.reference_angular_speed);
    p.reference_linear_acceleration =
        parameter(node, "reference_linear_acceleration", p.reference_linear_acceleration);
    p.reference_angular_acceleration =
        parameter(node, "reference_angular_acceleration", p.reference_angular_acceleration);
    p.tracking_position_limit =
        parameter(node, "tracking_position_limit", p.tracking_position_limit);
    p.tracking_orientation_limit =
        parameter(node, "tracking_orientation_limit", p.tracking_orientation_limit);
    impedance.cartesian_force_limit =
        parameter(node, "cartesian_force_limit", impedance.cartesian_force_limit);
    impedance.cartesian_moment_limit =
        parameter(node, "cartesian_moment_limit", impedance.cartesian_moment_limit);
    const double timeout = parameter(node, "command_timeout_ms", 100.0);
    const double uncertainty = parameter(node, "max_clock_uncertainty_ms", 10.0);
    const double future = parameter(node, "max_source_future_ms", 10.0);
    p.torque_rate = parameter(node, "maximum_torque_rate", p.torque_rate);
    p.tool_offset = parameter(node, "tool_offset_m", p.tool_offset);
    p.envelope_margin = parameter(node, "envelope_margin_m", p.envelope_margin);
    const auto center = parameter<std::vector<double>>(node, "envelope_center", {0, 0, 0.333});
    const auto limits =
        parameter<std::vector<double>>(node, "torque_limits", {87, 87, 87, 87, 12, 12, 12});
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 100 || !std::isfinite(uncertainty) ||
        uncertainty <= 0 || uncertainty > 10 || !std::isfinite(future) || future < 0 ||
        future > 10 || center.size() != 3 || limits.size() != 7)
      throw std::invalid_argument("invalid watchdog/clock/envelope/torque settings");
    p.timeout_ns = static_cast<uint64_t>(timeout * 1e6);
    p.uncertainty_ns = static_cast<uint64_t>(uncertainty * 1e6);
    p.future_ns = static_cast<uint64_t>(future * 1e6);
    p.envelope_center = Eigen::Map<const Eigen::Vector3d>(center.data());
    p.torque_limits = Eigen::Map<const ft::Vector7>(limits.data());
    if (!p.valid() || !impedance.valid() || frame.empty() ||
        (backend != "mock" && backend != "hardware") ||
        (controller_id != "left" && controller_id != "right") || !std::isfinite(trigger_release) ||
        trigger_release < 0 || trigger_release >= .75)
      throw std::invalid_argument("invalid controller parameters or backend");
    const bool hardware = backend == "hardware";
    if (hardware && (ip.empty() || !confirmed))
      throw std::invalid_argument("hardware backend requires robot_ip and operator_confirm:=true");
    local::ActuatorLimitScales scales;
    scales.torque = parameter(node, "torque_limit_scale", scales.torque);
    scales.torque_rate = parameter(node, "torque_rate_limit_scale", scales.torque_rate);
    if (!scales.valid())
      throw std::invalid_argument("controller torque scales must be finite and in (0, 1]");
    p.torque_limits = p.torque_limits.cwiseMin(local::panda_torque_limits(scales.torque));
    p.torque_rate = std::min(p.torque_rate, local::panda_torque_rate_limit(scales.torque_rate));
    const ft::Vector7 torque_limits = p.torque_limits;
    const double maximum_torque_rate = p.torque_rate;
    RCLCPP_INFO(node->get_logger(),
                "Local actuator limits: torque_scale=%.6g rate_scale=%.6g "
                "maximum_torque_rate_Nm_s=%.6g",
                scales.torque, scales.torque_rate, maximum_torque_rate);
    ft::HomingParameters hp;
    hp.home_joint_position = impedance.home_joint_position;
    hp.speed_factor = parameter(node, "homing_speed_factor", hp.speed_factor);
    hp.completion_timeout = parameter(node, "homing_timeout_seconds", hp.completion_timeout);
    hp.stop_duration = parameter(node, "homing_stop_duration_seconds", hp.stop_duration);
    hp.stop_acceleration = parameter(node, "homing_stop_acceleration", hp.stop_acceleration);
    hp.stop_timeout = parameter(node, "homing_stop_timeout_seconds", hp.stop_timeout);
    const auto hk = parameter<std::vector<double>>(
        node, "homing_stiffness",
        std::vector<double>(hp.stiffness.data(), hp.stiffness.data() + 7));
    const auto hd = parameter<std::vector<double>>(
        node, "homing_damping", std::vector<double>(hp.damping.data(), hp.damping.data() + 7));
    if (hk.size() != 7 || hd.size() != 7)
      throw std::invalid_argument("homing gains require seven values");
    hp.stiffness = Eigen::Map<const ft::Vector7>(hk.data());
    hp.damping = Eigen::Map<const ft::Vector7>(hd.data());
    hp.timeout_ns = p.timeout_ns;
    hp.uncertainty_ns = p.uncertainty_ns;
    hp.future_ns = p.future_ns;
    if (!hp.valid())
      throw std::invalid_argument("invalid homing parameters");
    ft::CartesianImpedanceController cartesian(impedance);
    ft::OrientationOnlyController orientation_only(impedance);
    ft::PositionHoldParameters hold;
    const auto hold_k = parameter<std::vector<double>>(
        node, "hold_stiffness",
        std::vector<double>(hold.stiffness.data(), hold.stiffness.data() + 7));
    const auto hold_d = parameter<std::vector<double>>(
        node, "hold_damping", std::vector<double>(hold.damping.data(), hold.damping.data() + 7));
    hold.stiffness_ramp_seconds =
        parameter(node, "hold_stiffness_ramp_seconds", hold.stiffness_ramp_seconds);
    if (hold_k.size() != 7 || hold_d.size() != 7)
      throw std::invalid_argument("hold_stiffness and hold_damping require seven values");
    hold.stiffness = Eigen::Map<const ft::Vector7>(hold_k.data());
    hold.damping = Eigen::Map<const ft::Vector7>(hold_d.data());
    if (!hold.valid())
      throw std::invalid_argument("invalid position hold gains or stiffness ramp");
    ft::PositionHoldController position_hold(hold);
    ft::DampingController damping(impedance.gain_translation, impedance.gain_rotation);
    ft::ControllerManager manager(p, hp, cartesian, orientation_only, position_hold, damping);
    const bool homing_enabled = parameter<bool>(node, "homing_enabled", true);
    local::SpscQueue<ft::HomingInput> home_inputs;
    local::SpscQueue<ft::Command> commands;
    local::LatestSnapshot<Telemetry> snapshots;
    local::SpscQueue<MotionDiagnostic, 4> motion_diagnostics;
    local::LatestSnapshot<PoseFeedback> feedback;
    auto tracking_queue = std::make_unique<local::SpscQueue<ft::TrackingSnapshot, 1024>>();
    local::LatestSnapshot<ft::TrackingSnapshot> live_tracking;
    std::atomic<uint64_t> tracking_drops{0};
    std::atomic<uint64_t> overflows{0};
    std::atomic<uint64_t> home_overflows{0};
    std::atomic<bool> control_requested{false};
    bool control_ready = false, command_seen = false;
    bool startup_wait_logged = false;
    const auto startup_watch_ns = monotonic_ns();
    std::array<bool, 3> last_command_flags{};
    bool raw_seen = false;
    uint64_t last_raw_log_ns = 0, last_command_log_ns = 0;
    std::array<bool, 9> last_raw_flags{};
    auto raw_input = node->create_subscription<quest_teleop_interfaces::msg::QuestControllerState>(
        raw_topic, rclcpp::QoS(20),
        [&](quest_teleop_interfaces::msg::QuestControllerState::ConstSharedPtr msg) {
          if (msg->controller_id != controller_id)
            return;
          const std::array<bool, 9> flags{msg->connection_valid,
                                          msg->session_focused,
                                          msg->pose_action_active,
                                          msg->position_valid && msg->position_tracked,
                                          msg->orientation_valid && msg->orientation_tracked,
                                          msg->trigger_active,
                                          msg->clock_sync_valid && msg->source_time_valid,
                                          msg->reference_space == "LOCAL",
                                          msg->trigger_value <= trigger_release};
          const auto diagnostic_ns = monotonic_ns();
          if ((!raw_seen || flags != last_raw_flags) &&
              (!raw_seen || diagnostic_ns - last_raw_log_ns >= 500000000)) {
            raw_seen = true;
            last_raw_flags = flags;
            last_raw_log_ns = diagnostic_ns;
            RCLCPP_INFO(node->get_logger(),
                        "Quest input: controller=%s connected=%d focused=%d pose_active=%d "
                        "position_ok=%d orientation_ok=%d trigger_active=%d clock_valid=%d "
                        "local_space=%d trigger=%.3f uncertainty_ms=%.3f",
                        controller_id.c_str(), flags[0], flags[1], flags[2], flags[3], flags[4],
                        flags[5], flags[6], flags[7], msg->trigger_value,
                        msg->clock_uncertainty_ns / 1e6);
          }
          if (!homing_enabled)
            return;
          ft::HomingInput c;
          c.receipt_ns = monotonic_ns();
          c.source_ns = msg->source_time_pc_monotonic_ns;
          c.sequence = msg->sequence;
          c.generation = msg->reference_space_generation;
          c.uncertainty_ns = msg->clock_uncertainty_ns;
          int matches = 0;
          bool button_active = false;
          for (const auto& button : msg->buttons)
            if (button.name == home_button) {
              ++matches;
              c.button_pressed = button.pressed;
              button_active = button.active;
            }
          c.valid = matches == 1 && button_active && msg->connection_valid &&
                    msg->session_focused && msg->pose_action_active && msg->position_valid &&
                    msg->orientation_valid && msg->position_tracked && msg->orientation_tracked &&
                    msg->trigger_active && std::isfinite(msg->trigger_value) &&
                    msg->trigger_value >= 0 && msg->trigger_value <= 1 && msg->source_time_valid &&
                    msg->clock_sync_valid && msg->reference_space == "LOCAL" &&
                    !msg->session_id.empty() && msg->session_id.size() <= 64 &&
                    msg->session_id.find('\0') == std::string::npos;
          c.trigger_released = msg->trigger_value <= trigger_release;
          if (c.valid)
            std::copy(msg->session_id.begin(), msg->session_id.end(), c.session.begin());
          if (!home_inputs.push(c))
            home_overflows.fetch_add(1, std::memory_order_release);
        });
    auto input = node->create_subscription<quest_teleop_interfaces::msg::CartesianTeleopCommand>(
        command_topic, rclcpp::QoS(10),
        [&](quest_teleop_interfaces::msg::CartesianTeleopCommand::ConstSharedPtr msg) {
          ft::Command c;
          c.receipt_ns = monotonic_ns();
          c.source_ns = msg->source_time_pc_monotonic_ns;
          c.uncertainty_ns = msg->clock_uncertainty_ns;
          c.sequence = msg->sequence;
          c.generation = msg->reference_space_generation;
          c.location_ns = msg->openxr_location_time_ns;
          c.enabled = msg->motion_enabled;
          c.orientation_only = msg->orientation_only;
          c.released = msg->trigger_released;
          c.valid = msg->header.frame_id == frame && msg->input_valid && msg->source_time_valid &&
                    msg->clock_sync_valid && !msg->session_id.empty() &&
                    msg->session_id.size() <= 64 && msg->session_id.find('\0') == std::string::npos;
          if (c.valid)
            std::copy(msg->session_id.begin(), msg->session_id.end(), c.session.begin());
          const auto& position = msg->target_pose.position;
          const auto& q = msg->target_pose.orientation;
          c.position << position.x, position.y, position.z;
          c.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
          const auto& t = msg->desired_twist;
          c.twist << t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z;
          const auto& requested = msg->requested_twist;
          c.requested_twist << requested.linear.x, requested.linear.y, requested.linear.z,
              requested.angular.x, requested.angular.y, requested.angular.z;
          c.requested_twist_valid = msg->requested_twist_valid && c.requested_twist.allFinite();
          if (!commands.push(c))
            overflows.fetch_add(1, std::memory_order_release);
          const std::array<bool, 3> flags{c.valid, c.released, c.enabled};
          if ((!command_seen || flags != last_command_flags) &&
              (!command_seen || c.receipt_ns - last_command_log_ns >= 500000000)) {
            command_seen = true;
            last_command_flags = flags;
            last_command_log_ns = c.receipt_ns;
            RCLCPP_INFO(node->get_logger(),
                        "Mapped input: valid=%d trigger_released=%d motion_requested=%d "
                        "frame_match=%d source_age_ms=%.3f uncertainty_ms=%.3f",
                        c.valid, c.released, c.enabled, msg->header.frame_id == frame,
                        c.source_ns && c.receipt_ns >= c.source_ns
                            ? (c.receipt_ns - c.source_ns) / 1e6
                            : -1.,
                        c.uncertainty_ns / 1e6);
          }
        });
    auto measured = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        pose_topic, rclcpp::SensorDataQoS().keep_last(1));
    auto joints = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    auto robot_state = node->create_publisher<franka_msgs::msg::FrankaState>(state_topic, 1);
    auto status = node->create_publisher<quest_teleop_interfaces::msg::ControllerStatus>(
        "controller_status", rclcpp::QoS(1).transient_local());
    auto home_status = node->create_publisher<quest_teleop_interfaces::msg::HomingStatus>(
        "homing_status", rclcpp::QoS(1).transient_local());
    auto tracking_samples = node->create_publisher<quest_teleop_interfaces::msg::TrackingSample>(
        "tracking_samples", rclcpp::QoS(2048));
    auto tracking_status = node->create_publisher<quest_teleop_interfaces::msg::TrackingSample>(
        "tracking_status", rclcpp::QoS(10));
    auto publish_tracking = [&](const ft::TrackingSnapshot& sample, const auto& publisher) {
      const auto now = monotonic_ns();
      const auto stamp = node->now() - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(
                                           now >= sample.state_ns ? now - sample.state_ns : 0));
      publisher->publish(
          ft::tracking_message(sample, frame, stamp, torque_limits, maximum_torque_rate,
                               tracking_drops.load(std::memory_order_relaxed), impedance));
    };
    // The mapper's freshness gate needs a current pose even when a reliable
    // status publisher blocks. Keep this small best-effort path with input;
    // bulk telemetry has its own executor and its own single-reader snapshot.
    auto feedback_timer = node->create_wall_timer(std::chrono::milliseconds(10), [&] {
      PoseFeedback sample;
      if (!feedback.read(sample)) {
        if (!control_ready && !startup_wait_logged &&
            monotonic_ns() - startup_watch_ns >= 5000000000ULL) {
          startup_wait_logged = true;
          RCLCPP_WARN(node->get_logger(),
                      "Arm startup still waiting for its first accepted torque cycle after "
                      "5 s: control_requested=%d robot_ip=%s. FCI startup mode Idle alone "
                      "does not confirm that torque callbacks have started.",
                      control_requested.load(std::memory_order_acquire), ip.c_str());
        }
        return;
      }
      if (!control_ready) {
        control_ready = true;
        RCLCPP_INFO(node->get_logger(),
                    "Arm control ready: accepted torque cycle=%llu robot_ip=%s "
                    "measured_frame=%s. Fresh valid trigger release/press is required "
                    "for Quest teleoperation; startup does not home the arm.",
                    static_cast<unsigned long long>(sample.cycle), ip.c_str(), frame.c_str());
      }
      const auto now = monotonic_ns();
      const auto stamp = node->now() - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(
                                           now >= sample.stamp_ns ? now - sample.stamp_ns : 0));
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame;
      write_pose(pose.pose, sample.position, sample.orientation);
      measured->publish(pose);
    });
    auto telemetry_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    std::string last_motion_report;
    uint64_t last_motion_log_ns = 0;
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(10),
        [&] {
          MotionDiagnostic diagnostic;
          for (int n = 0; n < 4 && motion_diagnostics.pop(diagnostic); ++n) {
            const auto path = "/tmp/franka-motion-model-arm-" + std::to_string(arm) + "-" +
                              std::to_string(diagnostic.stamp_ns) + ".txt";
            save_model_diagnostic(node->get_logger(), diagnostic.model, path);
            RCLCPP_ERROR(node->get_logger(), "Motion protection latched: cause=%s cycle=%llu",
                         ft::reason_name(diagnostic.reason),
                         static_cast<unsigned long long>(diagnostic.model.cycle));
            std::ofstream detail(path + ".motion.txt");
            detail.precision(17);
            detail << "cause " << ft::reason_name(diagnostic.reason) << "\nrequested_position "
                   << diagnostic.command.position.transpose() << "\nrequested_orientation_xyzw "
                   << diagnostic.command.orientation.coeffs().transpose() << "\nrequested_twist "
                   << diagnostic.command.twist.transpose() << "\nmeasured_torque "
                   << diagnostic.measured_torque.transpose() << "\ngravity "
                   << diagnostic.gravity.transpose() << '\n';
          }
          ft::TrackingSnapshot tracking;
          if (live_tracking.read(tracking))
            publish_tracking(tracking, tracking_status);
          Telemetry sample;
          if (!snapshots.read(sample))
            return;
          const auto now = monotonic_ns();
          const auto motion_report = std::string(ft::reason_name(sample.output.reason)) + "/" +
                                     ft::home_phase_name(sample.homing.phase);
          if (motion_report != last_motion_report &&
              (!last_motion_log_ns || now - last_motion_log_ns >= 500000000)) {
            last_motion_report = motion_report;
            last_motion_log_ns = now;
            RCLCPP_INFO(
                node->get_logger(),
                "Motion state: requested=%d enabled=%d controller=%s homing=%s "
                "position_error_m=%.6g joint_speed_rad_s=%.6g "
                "nominal_norm_Nm=%.6g command_norm_Nm=%.6g "
                "wrist_q_rad=[%.6g %.6g %.6g] wrist_dq_rad_s=[%.6g %.6g %.6g] "
                "wrist_nominal_Nm=[%.6g %.6g %.6g] wrist_commanded_Nm=[%.6g %.6g %.6g]",
                sample.motion_requested, sample.output.motion_enabled,
                ft::reason_name(sample.output.reason), ft::home_phase_name(sample.homing.phase),
                (sample.output.target_position - sample.state.position).norm(),
                sample.state.dq.cwiseAbs().maxCoeff(), sample.output.nominal.norm(),
                sample.output.torque.norm(), sample.state.q[4], sample.state.q[5],
                sample.state.q[6], sample.state.dq[4], sample.state.dq[5], sample.state.dq[6],
                sample.output.nominal[4], sample.output.nominal[5], sample.output.nominal[6],
                sample.output.torque[4], sample.output.torque[5], sample.output.torque[6]);
          }
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = node->now() - rclcpp::Duration::from_nanoseconds(
                                                now >= sample.stamp_ns ? now - sample.stamp_ns : 0);
          pose.header.frame_id = frame;
          sensor_msgs::msg::JointState joint;
          joint.header = pose.header;
          for (int j = 0; j < 7; ++j)
            joint.name.push_back("panda_joint" + std::to_string(j + 1));
          joint.position.assign(sample.state.q.data(), sample.state.q.data() + 7);
          joint.velocity.assign(sample.state.dq.data(), sample.state.dq.data() + 7);
          joints->publish(joint);
          franka_msgs::msg::FrankaState state;
          state.header = pose.header;
          std::copy_n(sample.state.q.data(), 7, state.q.begin());
          std::copy_n(sample.state.dq.data(), 7, state.dq.begin());
          state.tau_j = sample.native.tau_J;
          state.tau_j_d = sample.native.tau_J_d;
          state.dtau_j = sample.native.dtau_J;
          Eigen::Affine3d actual =
              Eigen::Translation3d(sample.state.position) * sample.state.orientation;
          Eigen::Affine3d target = Eigen::Translation3d(sample.output.target_position) *
                                   sample.output.target_orientation;
          Eigen::Map<Eigen::Matrix4d>(state.o_t_ee.data()) = actual.matrix();
          Eigen::Map<Eigen::Matrix4d>(state.o_t_ee_d.data()) = target.matrix();
          Eigen::Map<Eigen::Matrix4d>(state.o_t_ee_c.data()) = target.matrix();
          std::copy_n(sample.output.desired_twist.data(), 6, state.o_dp_ee_d.begin());
          state.time = sample.native.time.toSec();
          state.robot_mode = static_cast<uint8_t>(sample.native.robot_mode);
          robot_state->publish(state);
          quest_teleop_interfaces::msg::ControllerStatus event;
          event.header = pose.header;
          event.hardware_backend = hardware;
          event.motion_enabled = sample.output.motion_enabled;
          event.reason = ft::reason_name(sample.output.reason);
          event.source_time_pc_monotonic_ns = sample.output.source_ns;
          event.sequence = sample.output.sequence;
          event.command_queue_overflows = overflows.load(std::memory_order_acquire);
          event.torque_rate_limited = sample.output.rate_limited;
          event.torque_magnitude_limited = sample.output.magnitude_limited;
          std::copy_n(sample.output.nominal.data(), 7, event.nominal_torque.begin());
          std::copy_n(sample.output.torque.data(), 7, event.commanded_torque.begin());
          status->publish(event);
          quest_teleop_interfaces::msg::HomingStatus home;
          home.header = pose.header;
          home.controller_id = controller_id;
          home.phase = ft::home_phase_name(sample.homing.phase);
          home.message = sample.homing.message;
          home.request_id = sample.homing.request_id;
          home.active = sample.homing.active;
          home.stop_failed = sample.homing.stop_failed;
          home.elapsed_seconds = sample.homing.elapsed;
          home.maximum_joint_position_error = sample.homing.position_error;
          home.maximum_joint_velocity = sample.homing.velocity;
          std::copy_n(sample.homing.desired_q.data(), 7, home.desired_joint_position.begin());
          std::copy_n(sample.homing.desired_dq.data(), 7, home.desired_joint_velocity.begin());
          home_status->publish(home);
        },
        telemetry_group);
    prepared.fetch_add(1, std::memory_order_acq_rel);
    while (prepared.load(std::memory_order_acquire) < count &&
           !session.failed.load(std::memory_order_acquire))
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (session.failed.load(std::memory_order_acquire))
      throw std::runtime_error("peer setup failed");
    std::atomic<bool> running{true};
    // Create ROS guard conditions before hardware startup can fail. Constructing
    // rclcpp::spin's executor in the worker raced with shutdown during cleanup.
    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    rclcpp::executors::SingleThreadedExecutor telemetry_executor;
    telemetry_executor.add_callback_group(telemetry_group, node->get_node_base_interface());
    auto spin_executor = [&](rclcpp::executors::SingleThreadedExecutor& target) {
      try {
        while (running.load(std::memory_order_acquire) && rclcpp::ok())
          target.spin_once(std::chrono::milliseconds(10));
      } catch (const std::exception& e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "ROS executor stopped: %s", e.what());
          session.failed.store(true, std::memory_order_release);
        }
      }
      running.store(false, std::memory_order_release);
    };
    std::thread executor([&] { spin_executor(ros_executor); });
    // Dedicated consumer: full-rate ROS serialization cannot delay the command
    // executor or torque callback. A slow consumer loses samples with a counter.
    std::atomic<bool> monitoring_running{true};
    std::thread telemetry_thread;
    std::thread monitoring;
    try {
      telemetry_thread = std::thread([&] { spin_executor(telemetry_executor); });
      monitoring = std::thread([&] {
        try {
          while (true) {
            ft::TrackingSnapshot sample;
            const bool available = tracking_queue->pop(sample);
            if (available) {
              live_tracking.publish(sample);
              if (tracking_samples->get_subscription_count())
                publish_tracking(sample, tracking_samples);
            } else {
              if (!monitoring_running.load(std::memory_order_acquire))
                break;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "Monitoring publisher stopped: %s", e.what());
        }
      });
    } catch (...) {
      running.store(false, std::memory_order_release);
      ros_executor.cancel();
      telemetry_executor.cancel();
      if (telemetry_thread.joinable())
        telemetry_thread.join();
      executor.join();
      throw;
    }
    auto join_executor = [&] {
      monitoring_running.store(false, std::memory_order_release);
      if (monitoring.joinable())
        monitoring.join();
      running.store(false, std::memory_order_release);
      ros_executor.cancel();
      telemetry_executor.cancel();
      if (telemetry_thread.joinable())
        telemetry_thread.join();
      if (executor.joinable())
        executor.join();
    };
    uint64_t seen_overflows = 0;
    uint64_t seen_home_overflows = 0;
    uint64_t cycle = 0;
    bool protective_reported = false;
    ft::TrackingSnapshot previous_tracking;
    double shutdown_seconds = 0;
    auto step = [&](Telemetry& telemetry, double period) {
      ft::Command c;
      const auto count = overflows.load(std::memory_order_acquire);
      const bool overflow = count != seen_overflows;
      seen_overflows = count;
      for (int n = 0; n < 64 && commands.pop(c); ++n)
        if (!overflow && running.load(std::memory_order_acquire))
          manager.accept(c, monotonic_ns());
      if (overflow)
        manager.fault(ft::Reason::queue_overflow);
      const auto home_count = home_overflows.load(std::memory_order_acquire);
      const bool home_overflow = home_count != seen_home_overflows;
      seen_home_overflows = home_count;
      ft::HomingInput hi;
      for (int n = 0; n < 64 && home_inputs.pop(hi); ++n)
        if (!home_overflow)
          manager.homing_input(hi, monotonic_ns());
      if (home_overflow || overflow)
        manager.cancel_homing("command queue overflow");
      if (!running.load(std::memory_order_acquire) ||
          session.failed.load(std::memory_order_acquire)) {
        manager.shutdown();
      }
      telemetry.cycle = ++cycle;
      // ROS can enqueue input after the RobotState snapshot was timestamped.
      // Evaluate input watchdogs after draining the queues, so newly received
      // input cannot appear to arrive in the future. Keep the original model
      // timestamp for coherent tracking diagnostics.
      const auto input_check_ns = monotonic_ns();

      const auto managed = manager.update(telemetry.state, input_check_ns, period);
      telemetry.homing = managed.homing;
      telemetry.output = managed.control;
      telemetry.motion_requested = managed.control.motion_enabled;
      if (telemetry.output.reason == ft::Reason::invalid_state || telemetry.homing.stop_failed) {
        owner_fault = true;
        fault_period = period;
        fault_sample = telemetry;
        session.failed.store(true, std::memory_order_release);
        throw std::runtime_error(telemetry.homing.stop_failed
                                     ? "homing controlled stop failed"
                                     : "invalid arm state or actuator interval");
      }
      if (telemetry.output.protective_stop && !protective_reported) {
        protective_reported = true;
        MotionDiagnostic diagnostic;
        diagnostic.model.state = telemetry.model;
        diagnostic.model.state.q = telemetry.state.q;
        diagnostic.model.state.dq = telemetry.state.dq;
        diagnostic.model.state.coriolis = telemetry.state.coriolis;
        diagnostic.model.state.previous_torque = telemetry.state.previous_torque;
        diagnostic.model.nominal = telemetry.output.nominal;
        diagnostic.model.period = period;
        diagnostic.model.cycle = telemetry.cycle;
        diagnostic.model.arm = arm;
        diagnostic.stamp_ns = telemetry.stamp_ns;
        diagnostic.reason = telemetry.output.reason;
        diagnostic.command = manager.requested_command();
        diagnostic.measured_torque = Eigen::Map<const ft::Vector7>(telemetry.native.tau_J.data());
        diagnostic.gravity = telemetry.gravity;
        motion_diagnostics.push(diagnostic);
      }
      // Package-local actuator clipping applies to every nominal law and backend.
      const auto bounds = local::actuator_bounds(telemetry.state.previous_torque, torque_limits,
                                                 maximum_torque_rate, period);
      if (!bounds)
        throw std::runtime_error("invalid actuator interval");
      telemetry.output.torque = bounds->clamp(telemetry.output.nominal);
      const double slew = maximum_torque_rate * std::min(period, .001);
      telemetry.output.rate_limited =
          ((telemetry.output.nominal - telemetry.state.previous_torque).array().abs() > slew).any();
      telemetry.output.magnitude_limited =
          (telemetry.output.nominal.array().abs() > torque_limits.array()).any();
      if (!running.load(std::memory_order_acquire) ||
          session.failed.load(std::memory_order_acquire))
        shutdown_seconds += period;
      telemetry.finish_requested = shutdown_seconds >= .5 && !telemetry.homing.active;
      session.finishing[arm].store(telemetry.finish_requested, std::memory_order_release);
      snapshots.publish(telemetry);
      feedback.publish({telemetry.state.position, telemetry.state.orientation, telemetry.stamp_ns,
                        telemetry.cycle});
      ft::TrackingSnapshot tracking;
      tracking.state = telemetry.state;
      tracking.control = telemetry.output;
      tracking.requested = manager.requested_command();
      tracking.homing = telemetry.homing;
      tracking.robot_desired = telemetry.state.previous_torque;
      tracking.measured_torque = Eigen::Map<const ft::Vector7>(telemetry.native.tau_J.data());
      tracking.gravity = telemetry.gravity;
      tracking.cycle = telemetry.cycle;
      tracking.state_ns = telemetry.stamp_ns;
      tracking.control_ns = input_check_ns;
      tracking.issued_ns = monotonic_ns();
      tracking.period = period;
      tracking.hardware = hardware;
      tracking.previous_cycle = previous_tracking.cycle;
      tracking.previous_issued_ns = previous_tracking.issued_ns;
      tracking.previous_nominal = previous_tracking.control.nominal;
      tracking.previous_commanded = previous_tracking.control.torque;
      if (!tracking_queue->push(tracking))
        tracking_drops.fetch_add(1, std::memory_order_relaxed);
      previous_tracking = tracking;
    };
    try {
      RCLCPP_INFO(node->get_logger(), "%s backend; atomic input %s; starts disarmed",
                  backend.c_str(), command_topic.c_str());
      if (!hardware) {
        control_requested.store(true, std::memory_order_release);
        // Fixed fixture, not a dynamics, reachability, or collision model.
        Telemetry sample;
        sample.state.position << 0.4, 0, 0.4;
        sample.state.q << 0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854;
        sample.state.jacobian.leftCols<6>().setIdentity();
        sample.model = local::panda_model_fixture(sample.state.q);
        auto deadline = Clock::now();
        while (!(sample.finish_requested && session.may_finish())) {
          sample.stamp_ns = monotonic_ns();
          step(sample, 0.001);
          sample.state.previous_torque = sample.output.torque;
          deadline += std::chrono::milliseconds(1);
          std::this_thread::sleep_until(deadline);
        }
      } else {
        franka::Robot robot(ip);  // Enforces libfranka realtime requirements.
        franka::Model model = robot.loadModel();
        connected.fetch_add(1, std::memory_order_acq_rel);
        while (connected.load(std::memory_order_acquire) < count &&
               !session.failed.load(std::memory_order_acquire))
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (session.failed.load(std::memory_order_acquire))
          throw std::runtime_error("peer connection failed");
        const auto startup_state = robot.readOnce();
        std::ostringstream startup;
        startup << "FCI startup robot_ip=" << ip << " mode=" << startup_state.robot_mode
                << " current_errors=" << startup_state.current_errors
                << " last_motion_errors=" << startup_state.last_motion_errors;
        RCLCPP_INFO(node->get_logger(), "%s", startup.str().c_str());
        ft::require_idle_control_start(startup_state);
        control_requested.store(true, std::memory_order_release);
        robot.control(
            [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
              Telemetry sample;
              sample.stamp_ns = monotonic_ns();
              sample.native = state;
              const auto feedback = ft::franka_model_feedback(state, model);
              sample.state = feedback.control;
              sample.model = feedback.model;
              sample.gravity = feedback.gravity;
              step(sample, period.toSec() == 0 ? .001 : period.toSec());
              std::array<double, 7> torque{};
              std::copy_n(sample.output.torque.data(), 7, torque.begin());
              if (sample.finish_requested && session.may_finish())
                return franka::MotionFinished(franka::Torques(torque));
              return franka::Torques(torque);
            },
            true, franka::kMaxCutoffFrequency);
      }
    } catch (...) {
      join_executor();
      throw;
    }
    join_executor();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Controller stopped: %s", e.what());
    if (owner_fault) {
      // The executor has been joined before this catch; no logging in FCI callback.
      RCLCPP_ERROR(node->get_logger(),
                   "Owner fault cycle %llu: cause=%s period=%.9g homing_phase=%s homing_message=%s "
                   "request_id=%llu elapsed=%.6f home_error=%.9g reference_error=%.9g "
                   "velocity=%.9g stop_failed=%d",
                   static_cast<unsigned long long>(fault_sample.cycle),
                   ft::state_fault_name(fault_sample.output.state_fault), fault_period,
                   ft::home_phase_name(fault_sample.homing.phase), fault_sample.homing.message,
                   static_cast<unsigned long long>(fault_sample.homing.request_id),
                   fault_sample.homing.elapsed, fault_sample.homing.position_error,
                   (fault_sample.homing.desired_q - fault_sample.state.q).cwiseAbs().maxCoeff(),
                   fault_sample.homing.velocity, fault_sample.homing.stop_failed);
    }
    session.failed.store(true, std::memory_order_release);
    result = 2;
  }
  return result;
}

// Non-realtime diagnostic serialization, reusable by any future execution path.
void save_model_diagnostic(const rclcpp::Logger& logger, const local::ModelSnapshot& snapshot,
                           const std::string& path) {
  std::ofstream file(path);
  if (local::write_model_snapshot(file, snapshot))
    RCLCPP_WARN(logger, "Model snapshot saved to %s (column-major matrices)", path.c_str());
  else
    RCLCPP_ERROR(logger, "Could not save model snapshot to %s", path.c_str());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto root = std::make_shared<rclcpp::Node>("panda_cartesian_impedance_controller");
  int result = 2;
  try {
    const auto mode = parameter<std::string>(root, "mode", "single");
    const auto backend = parameter<std::string>(root, "backend", "mock");
    const bool hardware = backend == "hardware";
    if ((mode != "single" && mode != "dual") || (backend != "mock" && !hardware))
      throw std::invalid_argument("mode must be single/dual and backend mock/hardware");
    const bool confirmed = parameter(root, "operator_confirm", false);
    if (hardware && !confirmed)
      throw std::invalid_argument("hardware backend requires operator_confirm:=true");
    std::array<std::shared_ptr<rclcpp::Node>, 2> nodes;
    const int count = mode == "dual" ? 2 : 1;
    std::vector<std::string> ips, namespaces, frames;
    // Validate all ownership identities before opening any hardware connection.
    for (int i = 0; i < count; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      const std::string prefix = count == 2 ? side + "." : "";
      const std::string ns =
          count == 2 ? parameter<std::string>(root, prefix + "namespace", "/panda_" + side)
                     : root->get_namespace();
      const std::string id =
          parameter<std::string>(root, prefix + "controller_id", count == 2 ? side : "right");
      const std::string frame = parameter<std::string>(
          root, prefix + "base_frame",
          count == 2 ? "panda_" + side + "/panda_link0" : "panda_1/panda_link0");
      const std::string ip = parameter<std::string>(root, prefix + "robot_ip", "");
      if (frame.empty() || ns.empty() || ns.front() != '/' || (id != "left" && id != "right") ||
          (count == 2 && id != side) || (hardware && ip.empty()))
        throw std::invalid_argument(
            "invalid arm namespace, base frame, controller assignment or robot_ip");
      if (i && (ns == namespaces[0] || frame == frames[0] || (hardware && ip == ips[0])))
        throw std::invalid_argument("dual arm namespaces, frames and hardware IPs must be unique");
      namespaces.push_back(ns);
      frames.push_back(frame);
      ips.push_back(ip);
      std::vector<rclcpp::Parameter> overrides = {
          {"backend", backend},  {"operator_confirm", confirmed},
          {"robot_ip", ip},      {"base_frame", frame},
          {"controller_id", id}, {"homing_enabled", parameter<bool>(root, "homing_enabled", true)}};
      for (const auto& entry : std::vector<std::pair<std::string, std::string>>{
               {"command_topic", "cartesian_teleop_command"},
               {"measured_pose_topic", "teleop_anchor_pose"},
               {"state_topic", "robot_state"},
               {"controller_state_topic", "/quest_usb_bridge/" + id + "/controller_state"}})
        overrides.emplace_back(entry.first,
                               parameter<std::string>(root, prefix + entry.first, entry.second));
      overrides.emplace_back("nullspace_enabled",
                             parameter(root, prefix + "nullspace_enabled", true));
      for (const auto* name : {"reference_shaping_enabled", "tracking_protection_enabled",
                               "cartesian_wrench_limits_enabled"})
        overrides.emplace_back(name, parameter(root, prefix + name, false));
      for (const auto& entry :
           std::vector<std::pair<std::string, double>>{{"nullspace_stiffness", 10.0},
                                                       {"nullspace_damping", 6.324555320336759},
                                                       {"nullspace_joint_limit_gain", 2.0},
                                                       {"nullspace_joint_limit_margin", .35},
                                                       {"nullspace_max_torque", 5.0},
                                                       {"nullspace_ramp_seconds", .25},
                                                       {"reference_linear_speed", 0.05},
                                                       {"reference_angular_speed", 0.15},
                                                       {"reference_linear_acceleration", 0.15},
                                                       {"reference_angular_acceleration", 0.5},
                                                       {"tracking_position_limit", 0.03},
                                                       {"tracking_orientation_limit", 0.2},
                                                       {"cartesian_force_limit", 10.0},
                                                       {"cartesian_moment_limit", 1.5},
                                                       {"lambda_translation", 50},
                                                       {"lambda_rotation", 40},
                                                       {"gain_translation", 20},
                                                       {"gain_rotation", 2.5},
                                                       {"command_timeout_ms", 100},
                                                       {"max_clock_uncertainty_ms", 10},
                                                       {"max_source_future_ms", 10},
                                                       {"maximum_torque_rate", 900},
                                                       {"torque_limit_scale", .35},
                                                       {"torque_rate_limit_scale", .35},
                                                       {"tool_offset_m", .1034},
                                                       {"envelope_margin_m", .001},
                                                       {"trigger_release", .25},
                                                       {"homing_speed_factor", .15},
                                                       {"homing_timeout_seconds", 30},
                                                       {"homing_stop_duration_seconds", .5},
                                                       {"homing_stop_acceleration", .75},
                                                       {"homing_stop_timeout_seconds", 5},
                                                       {"hold_stiffness_ramp_seconds", .25}}) {
        const double value = parameter(root, prefix + entry.first, entry.second);
        overrides.emplace_back(entry.first, value);
      }
      for (const auto& entry : std::vector<std::pair<std::string, std::vector<double>>>{
               {"home_joint_position",
                std::vector<double>(local::kHome.begin(), local::kHome.end())},
               {"envelope_center", {0, 0, .333}},
               {"torque_limits", {87, 87, 87, 87, 12, 12, 12}},
               {"homing_stiffness", {300, 300, 300, 300, 120, 100, 50}},
               {"homing_damping", {30, 30, 30, 30, 10, 10, 5}},
               {"hold_stiffness", {600, 600, 600, 600, 250, 150, 75}},
               {"hold_damping", {50, 50, 50, 50, 20, 15, 10}}}) {
        const auto values = parameter(root, prefix + entry.first, entry.second);
        overrides.emplace_back(entry.first, values);
      }
      nodes[i] = count == 1
                     ? root
                     : std::make_shared<rclcpp::Node>(
                           "panda_cartesian_impedance_controller", ns,
                           rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(
                               overrides));
    }
    for (const auto& entry : root->get_node_parameters_interface()->get_parameter_overrides())
      if (!root->has_parameter(entry.first))
        throw std::invalid_argument("unknown controller parameter: " + entry.first);
    if (hardware && mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
      throw std::runtime_error("hardware control requires memory locking");
    RuntimeSession session;
    session.arms = count;
    std::atomic<unsigned> prepared{0}, connected{0};
    std::array<int, 2> results{};
    std::array<std::thread, 2> owners;
    try {
      for (int i = 0; i < count; ++i)
        owners[i] = std::thread(
            [&, i] { results[i] = run_arm(nodes[i], i, session, prepared, connected, count); });
      for (int i = 0; i < count; ++i)
        owners[i].join();
    } catch (...) {
      session.failed.store(true, std::memory_order_release);
      for (auto& owner : owners)
        if (owner.joinable())
          owner.join();
      throw;
    }
    result = results[0] || results[1] ? 2 : 0;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(root->get_logger(), "Controller setup stopped: %s", e.what());
  }
  rclcpp::shutdown();
  return result;
}
