#include <franka/control_tools.h>
#include <franka/control_types.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <pthread.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <franka_msgs/msg/franka_state.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <quest_teleop_interfaces/msg/cartesian_teleop_command.hpp>
#include <quest_teleop_interfaces/msg/controller_status.hpp>
#include <quest_teleop_interfaces/msg/homing_status.hpp>
#include <quest_teleop_interfaces/msg/gripper_width.hpp>
#include <quest_teleop_interfaces/msg/quest_controller_state.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <thread>

#include "cbf_diagnostics.hpp"
#include "cbf_safety_filter/safety_filter.hpp"
#include "hardware_startup.hpp"
#include "teleop_controllers/cartesian_pd_controller.hpp"
#include "teleop_controllers/controller_manager.hpp"
#include "teleop_controllers/damping_controller.hpp"
#include "teleop_controllers/franka_model.hpp"
#include "teleop_controllers/handoff.hpp"
#include "teleop_controllers/inverse_dynamics_controller.hpp"
#include "teleop_controllers/inter_arm_policy.hpp"
#include "teleop_controllers/model_snapshot.hpp"
#include "teleop_controllers/orientation_only_controller.hpp"
#include "teleop_controllers/position_hold_controller.hpp"
#include "teleop_controllers/urdf_model.hpp"
#include "tracking_message.hpp"

namespace cbf = cbf_safety_filter;
namespace ft = teleop_controllers;
namespace local = teleop_controllers;
using Clock = std::chrono::steady_clock;
constexpr const char* kUrdfResource = "package://teleop_controllers/config/panda_oscbf.urdf";
std::string model_path(const std::string& resource) {
  if (resource.rfind("package://", 0) == 0) {
    const auto separator = resource.find('/', 10);
    if (separator == std::string::npos || separator == 10 || separator + 1 == resource.size())
      throw std::invalid_argument("invalid dynamics_model_file package resource");
    return (std::filesystem::path(
                ament_index_cpp::get_package_share_directory(resource.substr(10, separator - 10))) /
            resource.substr(separator + 1))
        .string();
  }
  if (resource.empty())
    throw std::invalid_argument("dynamics_model_file cannot be empty");
  return std::filesystem::absolute(resource).string();
}
uint64_t monotonic_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
}
uint64_t thread_cpu_ns() noexcept {
  timespec t{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  return uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
// Explicit mock qualification option. Children/executors are created first,
// so only this owner inherits the same FIFO policy as libfranka control.
struct MockPriority {
  bool active{false};
  explicit MockPriority(bool enabled) {
    if (!enabled)
      return;
    std::string error;
    if (!franka::setCurrentThreadToHighestSchedulerPriority(&error))
      throw std::runtime_error("mock real-time scheduling required: " + error);
    active = true;
  }
  ~MockPriority() {
    if (active) {
      sched_param p{};
      pthread_setschedparam(pthread_self(), SCHED_OTHER, &p);
    }
  }
};
struct Telemetry {
  ft::FciTiming fci;
  ft::ArmState state;
  ft::Output output;
  ft::HomingOutput homing;
  franka::RobotState native;
  local::RobotModelState model;
  ft::Vector7 gravity{ft::Vector7::Zero()};
  double wake_lateness_us{0}, observed_period_us{0};
  bool finish_requested{false};
  bool motion_requested{false};
  uint64_t cpu_start_ns{0};
  double model_acquisition_us{0};
  uint64_t stamp_ns{0}, cycle{0};
};
struct CbfDiagnostic {
  ft::FciTiming fci;
  cbf::Result result;
  cbf::CommandInspection nominal_inspection;
  ft::Vector7 command{ft::Vector7::Zero()};
  uint64_t cycle{0}, state_ns{0};
  double callback_us{0}, cpu_us{0}, wake_lateness_us{0}, observed_period_us{0};
  bool motion_enabled{false};
  double model_acquisition_us{0};
  bool actual_command_verified{false};
  bool held_local_correction_verified{false};
  bool inter_arm_enabled{false}, peer_valid{false};
  uint64_t peer_state_ns{0};
  double peer_age_us{0};
  bool native_recovery_qp{false};
  double table_height_m{0}, table_clearance_m{0};
};
struct CbfRejected {
  cbf::Status status;
};
struct PoseFeedback {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{1, 0, 0, 0};
  uint64_t stamp_ns{0}, cycle{0};
};
// Lifecycle and peer snapshots for independently filtered arm owners.
struct RuntimeSession {
  std::atomic<bool> failed{false};
  std::atomic<bool> protective_stop{false};
  std::array<std::atomic<bool>, 2> finishing{};
  int arms{1};
  std::array<std::unique_ptr<ft::LatestSnapshot<cbf::PeerSnapshot>>, 2> collision_exchange;
  std::array<std::atomic<uint64_t>, 2> startup_seed_ns{};
  bool may_finish() const noexcept {
    if (failed.load(std::memory_order_acquire))
      return true;  // A failed owner cannot acknowledge completion.
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
  if (name == "cbf_enabled" || name == "cbf_config_path" ||
      name.find(".cbf_enabled") != std::string::npos ||
      name.find(".cbf_config_path") != std::string::npos || name == "cbf_commissioning_mode" ||
      name.find(".cbf_commissioning_mode") != std::string::npos ||
      name.find(".cbf_commissioning_") != std::string::npos) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Immutable CBF startup selection; restart to revalidate changes";
    return node->declare_parameter<T>(name, value, descriptor);
  }
  return node->declare_parameter<T>(name, value);
}
int run_arm(const std::shared_ptr<rclcpp::Node>& node, int arm, RuntimeSession& session,
            std::atomic<unsigned>& prepared, std::atomic<unsigned>& connected, unsigned count) {
  int result = 0;
  bool owner_fault = false;
  double fault_period = 0;
  Telemetry fault_sample;
  cbf::Status cbf_fault = cbf::Status::disabled;
  try {
    const auto backend = parameter<std::string>(node, "backend", "mock");
    const ft::UrdfPandaModel dynamics(
        model_path(parameter<std::string>(node, "dynamics_model_file", kUrdfResource)));
    RCLCPP_INFO(node->get_logger(), "Dynamics model=oscbf_urdf file=%s fnv1a64=%s",
                dynamics.filename().c_str(), dynamics.fingerprint().c_str());
    const auto ip = parameter<std::string>(node, "robot_ip", "");
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
    ft::CartesianPdParameters pd;
    pd.lambda_translation =
        parameter(node, "lambda_translation", pd.lambda_translation);
    pd.lambda_rotation = parameter(node, "lambda_rotation", pd.lambda_rotation);
    pd.gain_translation = parameter(node, "gain_translation", pd.gain_translation);
    pd.gain_rotation = parameter(node, "gain_rotation", pd.gain_rotation);
    const auto home = parameter<std::vector<double>>(
        node, "home_joint_position", std::vector<double>(local::kHome.begin(), local::kHome.end()));
    if (home.size() != 7)
      throw std::invalid_argument("home_joint_position requires seven values");
    pd.home_joint_position = Eigen::Map<const ft::Vector7>(home.data());
    pd.nullspace_stiffness =
        parameter(node, "nullspace_stiffness", pd.nullspace_stiffness);
    pd.nullspace_damping = parameter(node, "nullspace_damping", pd.nullspace_damping);
    pd.nullspace_joint_limit_gain =
        parameter(node, "nullspace_joint_limit_gain", pd.nullspace_joint_limit_gain);
    pd.nullspace_joint_limit_margin =
        parameter(node, "nullspace_joint_limit_margin", pd.nullspace_joint_limit_margin);
    pd.nullspace_max_torque =
        parameter(node, "nullspace_max_torque", pd.nullspace_max_torque);
    pd.nullspace_ramp_seconds =
        parameter(node, "nullspace_ramp_seconds", pd.nullspace_ramp_seconds);
    p.reference_linear_speed = parameter(node, "reference_linear_speed", p.reference_linear_speed);
    p.reference_shaping_enabled =
        parameter(node, "reference_shaping_enabled", p.reference_shaping_enabled);
    p.tracking_protection_enabled =
        parameter(node, "tracking_protection_enabled", p.tracking_protection_enabled);
    pd.cartesian_wrench_limits_enabled = parameter(
        node, "cartesian_wrench_limits_enabled", pd.cartesian_wrench_limits_enabled);
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
    pd.cartesian_force_limit =
        parameter(node, "cartesian_force_limit", pd.cartesian_force_limit);
    pd.cartesian_moment_limit =
        parameter(node, "cartesian_moment_limit", pd.cartesian_moment_limit);
    const auto control_law =
        parameter<std::string>(node, "cartesian_control_law", "cartesian_pd");
    if (control_law != "cartesian_pd" && control_law != "inverse_dynamics")
      throw std::invalid_argument(
          "cartesian_control_law must be cartesian_pd/inverse_dynamics");
    ft::InverseDynamicsParameters inverse;
    inverse.common = pd;
    const auto stiffness =
        parameter<std::vector<double>>(node, "inverse_dynamics_stiffness",
                                       {pd.lambda_translation * pd.gain_translation,
                                        pd.lambda_translation * pd.gain_translation,
                                        pd.lambda_translation * pd.gain_translation,
                                        pd.lambda_rotation * pd.gain_rotation,
                                        pd.lambda_rotation * pd.gain_rotation,
                                        pd.lambda_rotation * pd.gain_rotation});
    inverse.damping_ratio = parameter(node, "inverse_dynamics_damping_ratio", 1.0);
    const auto posture_k = parameter<std::vector<double>>(
        node, "inverse_dynamics_nullspace_stiffness", std::vector<double>(7, 10.));
    const auto posture_d = parameter<std::vector<double>>(
        node, "inverse_dynamics_nullspace_damping", std::vector<double>(7, 6.324555320336759));
    if (stiffness.size() != 6 || posture_k.size() != 7 || posture_d.size() != 7)
      throw std::invalid_argument("inverse dynamics requires six task and seven nullspace gains");
    inverse.stiffness = Eigen::Map<const ft::Vector6>(stiffness.data());
    inverse.nullspace_stiffness = Eigen::Map<const ft::Vector7>(posture_k.data());
    inverse.nullspace_damping = Eigen::Map<const ft::Vector7>(posture_d.data());
    if (!inverse.valid())
      throw std::invalid_argument("invalid inverse dynamics gains");
    const double timeout = parameter(node, "command_timeout_ms", 100.0);
    const double uncertainty = parameter(node, "max_clock_uncertainty_ms", 10.0);
    const double future = parameter(node, "max_source_future_ms", 10.0);
    p.torque_rate = parameter(node, "maximum_torque_rate", p.torque_rate);
    const auto limits =
        parameter<std::vector<double>>(node, "torque_limits", {87, 87, 87, 87, 12, 12, 12});
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 100 || !std::isfinite(uncertainty) ||
        uncertainty <= 0 || uncertainty > 10 || !std::isfinite(future) || future < 0 ||
        future > 10 || limits.size() != 7)
      throw std::invalid_argument("invalid clock/torque settings");
    p.timeout_ns = static_cast<uint64_t>(timeout * 1e6);
    p.uncertainty_ns = static_cast<uint64_t>(uncertainty * 1e6);
    p.future_ns = static_cast<uint64_t>(future * 1e6);
    p.torque_limits = Eigen::Map<const ft::Vector7>(limits.data());
    if (!p.valid() || !pd.valid() || frame.empty() ||
        (backend != "mock" && backend != "hardware") ||
        (controller_id != "left" && controller_id != "right") || !std::isfinite(trigger_release) ||
        trigger_release < 0 || trigger_release >= .75)
      throw std::invalid_argument("invalid controller parameters or backend");
    const bool hardware = backend == "hardware";
    const bool mock_realtime = parameter(node, "mock_realtime", false);
    const bool cbf_enabled = parameter(node, "cbf_enabled", false);
    const auto cbf_path = parameter<std::string>(node, "cbf_config_path", "");
    const bool cbf_shadow =
        cbf_enabled && parameter<std::string>(node, "cbf_commissioning_mode", "filtered") == "shadow";
    std::unique_ptr<cbf::SafetyFilter> cbf_filter;
    cbf::Configuration selected_cbf;
    if (cbf_enabled) {
      const auto configuration = cbf::load_configuration(cbf_path, controller_id);
      selected_cbf = configuration;
      if (hardware && (!configuration.diagnostic_times || !configuration.diagnostic_residuals ||
                       configuration.max_callback_time_us > 800.))
        throw std::invalid_argument(
            "hardware commissioning requires timing/residual diagnostics and callback "
            "budget <=800 us");
      cbf_filter = std::make_unique<cbf::SafetyFilter>(configuration);
      RCLCPP_WARN(node->get_logger(), "CBF output mode=%s; hardware commissioning is experimental",
                  cbf_shadow ? "shadow (nominal torque issued; no CBF protection)" : "filtered");
      if (configuration.inter_arm.enabled) {
        if (count != 2)
          throw std::invalid_argument("inter-arm filtering requires two enabled participants");
        session.collision_exchange[arm] = std::make_unique<ft::LatestSnapshot<cbf::PeerSnapshot>>();
      }
      RCLCPP_INFO(node->get_logger(),
                  "CBF filter: backend=%s max_rows=%d hardware_qualified=0 warm_start_effective=%d",
                  cbf::name(configuration.backend), configuration.max_constraints,
                  configuration.warm_start && configuration.backend != cbf::Backend::piqp);
    }
    if (hardware && ip.empty())
      throw std::invalid_argument("hardware backend requires robot_ip");
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
    hp.home_joint_position = pd.home_joint_position;
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
    ft::CartesianPdController cartesian(pd);
    ft::OrientationOnlyController orientation_only(pd);
    ft::InverseDynamicsController inverse_cartesian(inverse);
    ft::InverseDynamicsController inverse_orientation(inverse);
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
    ft::DampingController damping(pd.gain_translation, pd.gain_rotation);
    const bool inverse_selected = control_law == "inverse_dynamics";
    ft::TorqueController& teleoperation =
        inverse_selected ? static_cast<ft::TorqueController&>(inverse_cartesian) : cartesian;
    ft::TorqueController& orientation =
        inverse_selected ? static_cast<ft::TorqueController&>(inverse_orientation)
                         : orientation_only;
    ft::ControllerManager manager(p, hp, teleoperation, orientation, position_hold, damping);
    RCLCPP_INFO(node->get_logger(), "Cartesian control law=%s", control_law.c_str());
    const bool homing_enabled = parameter<bool>(node, "homing_enabled", true);
    local::SpscQueue<ft::HomingInput> home_inputs;
    local::SpscQueue<ft::Command> commands;
    local::LatestSnapshot<Telemetry> snapshots;
    local::LatestSnapshot<cbf::FingerOpening> finger_opening_exchange;
    cbf::FingerOpening finger_opening;
    rclcpp::Subscription<quest_teleop_interfaces::msg::GripperWidth>::SharedPtr width_subscription;
    if (cbf_filter && cbf_filter->configuration().uses_body_volumes) {
      width_subscription = node->create_subscription<quest_teleop_interfaces::msg::GripperWidth>(
          "gripper_width", rclcpp::QoS(1).best_effort(),
          [&](const quest_teleop_interfaces::msg::GripperWidth& sample) {
            finger_opening_exchange.publish({sample.width_m, sample.sample_time_pc_monotonic_ns});
          });
    }
    local::SpscQueue<MotionDiagnostic, 4> motion_diagnostics;
    std::unique_ptr<local::SpscQueue<CbfDiagnostic>> cbf_diagnostics;
    std::unique_ptr<ft::CbfTerminalSnapshot> terminal_snapshot;
    std::atomic<uint64_t> cbf_drops{0};
    std::atomic<bool> terminal_snapshot_pending{false};
    if (cbf_filter) {
      cbf_diagnostics = std::make_unique<local::SpscQueue<CbfDiagnostic>>();
      terminal_snapshot = std::make_unique<ft::CbfTerminalSnapshot>();
    }
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
                               tracking_drops.load(std::memory_order_relaxed), pd));
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cbf_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cbf_rejection_publisher;
    rclcpp::TimerBase::SharedPtr cbf_timer;
    if (cbf_filter) {
      cbf_publisher = node->create_publisher<std_msgs::msg::String>("cbf_status", 10);
      cbf_rejection_publisher = node->create_publisher<std_msgs::msg::String>(
          "cbf_rejection", rclcpp::QoS(1).reliable().transient_local());
      cbf_timer = node->create_wall_timer(
          std::chrono::milliseconds(10),
          [&] {
            if (terminal_snapshot_pending.exchange(false, std::memory_order_acq_rel)) {
              std::ostringstream json;
              ft::write_cbf_terminal_snapshot(json, *terminal_snapshot, dynamics.fingerprint());
              const auto payload = json.str();
              RCLCPP_ERROR(node->get_logger(), "CBF_PROTECTIVE_STOP_SNAPSHOT %s",
                           payload.c_str());
              std_msgs::msg::String message;
              message.data = payload;
              cbf_rejection_publisher->publish(message);
            }
            CbfDiagnostic sample;
            for (int i = 0; i < 64 && cbf_diagnostics->pop(sample); ++i) {
              const auto& r = sample.result;
              const auto& config = cbf_filter->configuration();
              std::ostringstream json;
              json.precision(17);
              const auto number = [&](double value) {
                if (std::isfinite(value))
                  json << value;
                else
                  json << "null";
              };
              json << "{\"enabled\":true,\"status\":\"" << cbf::name(r.status)
                   << "\",\"backend\":\"" << cbf::name(r.backend)
                   << "\",\"accepted\":" << (r.accepted() ? "true" : "false")
                   << ",\"cycle\":" << sample.cycle << ",\"model_timestamp_ns\":" << sample.state_ns
                   << ",\"rows\":" << r.rows
                   << ",\"group_pruned_pairs\":" << r.group_pruned_pairs
                   << ",\"evaluated_peer_pairs\":" << r.evaluated_peer_pairs
                                      << ",\"output_mode\":\"" << (cbf_shadow ? "shadow" : "filtered") << "\""
                   << ",\"actual_command_verified\":"
                   << (sample.actual_command_verified ? "true" : "false")
                   << ",\"held_local_correction_verified\":"
                   << (sample.held_local_correction_verified ? "true" : "false")
                   << ",\"nominal_domain_valid\":"
                   << (sample.nominal_inspection.domain_valid ? "true" : "false")
                   << ",\"nominal_command_feasible\":"
                   << (sample.nominal_inspection.feasible() ? "true" : "false")
                   << ",\"nominal_max_actuator_violation\":"
                   << sample.nominal_inspection.max_actuator_violation
                   << ",\"nominal_max_row_violation\":"
                   << sample.nominal_inspection.max_row_violation
                   << ",\"nominal_worst_family\":" << sample.nominal_inspection.worst_family
                   << ",\"current_box_infeasible_rows\":"
                   << sample.nominal_inspection.impossible_rows << ",\"current_box_worst_family\":"
                   << sample.nominal_inspection.impossible_family
                   << ",\"current_box_max_min_row_violation\":"
                   << sample.nominal_inspection.max_box_min_row_violation
                   << ",\"proposal_residuals_valid\":" << (r.accepted() ? "true" : "false")
                   << ",\"wake_lateness_valid\":" << (hardware ? "false" : "true")
                   << ",\"motion_enabled\":" << (sample.motion_enabled ? "true" : "false")
                   << ",\"algorithm\":\""
                   << (sample.native_recovery_qp ? "bounded_recovery_native_qp"
                                                 : "bounded_recovery_projection")
                   << '"' << ",\"table_height_m\":" << sample.table_height_m
                   << ",\"table_clearance_m\":" << sample.table_clearance_m
                   << ",\"recovery_policy\":\"bounded_recovery\""
                   << ",\"recovery_rows\":" << r.recovery_rows
                   << ",\"outside_domain_rows\":" << r.outside_domain_rows
                   << ",\"max_requested_violation\":" << r.max_requested_violation
                   << ",\"requested_qp_status\":\"" << cbf::name(r.requested_qp_status) << '"'
                   << ",\"recovery_qp_status\":\"" << cbf::name(r.recovery_qp_status) << '"'
                   << ",\"recovery_projection_fallback\":"
                   << (r.recovery_projection_fallback ? "true" : "false")
                   << ",\"inter_arm_enabled\":" << (sample.inter_arm_enabled ? "true" : "false")
                   << ",\"peer_valid\":" << (sample.peer_valid ? "true" : "false")
                   << ",\"peer_state_ns\":" << sample.peer_state_ns
                   << ",\"peer_age_us\":" << sample.peer_age_us
                   << ",\"analytic_solution\":" << (r.analytic_solution ? "true" : "false")
                   << ",\"dynamics_model\":\"oscbf_urdf\""
                   << ",\"dynamics_model_fnv1a64\":\"" << dynamics.fingerprint() << "\""
                   << ",\"iterations\":" << r.iterations
                   << ",\"dropped_samples\":" << cbf_drops.load();
              for (const auto& field : std::initializer_list<std::pair<const char*, double>>{
                       {"max_hard_violation", r.max_hard_violation},
                       {"intervention_norm", r.intervention_norm}}) {
                json << ",\"" << field.first << "\":";
                number(field.second);
              }
              if (config.diagnostic_times)
                for (const auto& field : std::initializer_list<std::pair<const char*, double>>{
                         {"model_us", r.model_us},
                         {"model_acquisition_us", sample.model_acquisition_us},
                         {"kinematics_us", r.kinematics_us},
                         {"constraints_us", r.constraints_us},
                         {"assembly_us", r.assembly_us},
                         {"update_us", r.update_us},
                         {"solver_us", r.solve_us},
                         {"verify_us", r.verify_us},
                         {"total_us", r.total_us},
                         {"controller_callback_us", sample.callback_us},
                         {"controller_cpu_us", sample.cpu_us},
                         {"wake_lateness_us", sample.wake_lateness_us},
                         {"observed_period_us", sample.observed_period_us}}) {
                  json << ",\"" << field.first << "\":";
                  number(field.second);
                }
              json << ",\"fci_timing\":";
              ft::write_fci_timing(json,sample.fci);
              json << ",\"candidate_torque\":[";
              for (int j = 0; j < 7; ++j) {
                if (j)
                  json << ',';
                number(r.torque[j]);
              }
              json << "],\"commanded_torque\":[";
              for (int j = 0; j < 7; ++j) {
                if (j)
                  json << ',';
                number(sample.command[j]);
              }
              json << ']';
              if (config.diagnostic_residuals) {
                json << ",\"constraints\":[";
                for (int row = 0; row < r.rows; ++row) {
                  if (row)
                    json << ',';
                  json << "{\"family\":" << static_cast<int>(r.families[row]);
                  if (config.diagnostic_residuals) {
                    json << ",\"original_residual\":";
                    number(r.original_residual[row]);
                  }
                  json << '}';
                }
                json << ']';
              }
              json << '}';
              std_msgs::msg::String message;
              message.data = json.str();
              cbf_publisher->publish(message);
            }
          },
          telemetry_group);
    }
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
    if (count == 2 && bool(session.collision_exchange[0]) != bool(session.collision_exchange[1]))
      throw std::invalid_argument("inter-arm filtering must be enabled on both arms");
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
    struct ProjectionContext {
      cbf::SafetyFilter* local;
    } projection_context{cbf_filter.get()};
    cbf::PeerSnapshot peer_snapshot;
    struct CollisionHandoff {
      RuntimeSession* session;
      int arm;
      cbf::PeerSnapshot* peer;
      bool recovery;
      cbf::PeerSnapshot incoming;
      void read() noexcept {
        if (!recovery) { session->collision_exchange[1 - arm]->read(*peer); return; }
        if (session->collision_exchange[1 - arm]->read(incoming) &&
            ft::valid_collision_snapshot(incoming) && incoming.timestamp_ns <= monotonic_ns() &&
            incoming.timestamp_ns >= peer->timestamp_ns) *peer = incoming;
      }
    } collision_handoff{&session, arm, &peer_snapshot,
                         cbf_filter != nullptr, {}};
    const cbf::PeerExchange collision_exchange{
        &collision_handoff,
        [](void* context, const cbf::PeerSnapshot& own) noexcept {
          auto& h = *static_cast<CollisionHandoff*>(context);
          h.session->collision_exchange[h.arm]->publish(own);
          h.read();
          return static_cast<const cbf::PeerSnapshot*>(h.peer);
        },
        [](void* context) noexcept {
          auto& h = *static_cast<CollisionHandoff*>(context);
          h.read();
          return static_cast<const cbf::PeerSnapshot*>(h.peer);
        }};
    uint64_t control_request_ns = 0;
    uint64_t startup_observation_count = 0, last_startup_observation_ns = 0;
    bool filter_prepared_this_cycle = false;
    const auto reject_cbf = [&](cbf::Status status, const char* stage, Telemetry& sample,
                                const ft::ActuatorBounds& bounds, double period,
                                bool nominal_available) {
      // Fixed-size owner-thread copy only. Formatting and ROS publication run on
      // the timer thread, never in the torque callback. Retain first-cycle faults.
      auto& terminal = *terminal_snapshot;
      terminal.present = true;
      terminal.status = status;
      terminal.stage = stage;
      terminal.failure = cbf_filter->constraint_failure();
      terminal.result = cbf_filter->last_result();
      terminal.bounds = {bounds.lower, bounds.upper};
      terminal.peer = peer_snapshot;
      terminal.control_request_ns = control_request_ns;
      terminal.own_startup_seed_ns = session.startup_seed_ns[arm].load(std::memory_order_acquire);
      terminal.peer_startup_seed_ns = session.startup_seed_ns[1 - arm].load(std::memory_order_acquire);
      terminal.startup_observation_count = startup_observation_count;
      terminal.last_startup_observation_ns = last_startup_observation_ns;
      terminal.model = sample.model;
      terminal.model.previous_torque = sample.state.previous_torque;
      terminal.cycle = sample.cycle;
      terminal.state_ns = sample.stamp_ns;
      terminal.period = period;
      terminal.observed_period_us = sample.observed_period_us;
      terminal.model_acquisition_us = sample.model_acquisition_us;
      terminal.fci = sample.fci;
      terminal.nominal_available = nominal_available;
      terminal.filter_timing_valid = filter_prepared_this_cycle;
      terminal.nominal = sample.output.nominal;
      terminal.measured_torque = Eigen::Map<const ft::Vector7>(sample.native.tau_J.data());
      terminal.gravity = sample.gravity;
      terminal.previous_command_valid = previous_tracking.cycle != 0;
      terminal.previous_cycle = previous_tracking.cycle;
      terminal.previous_issue_ns = previous_tracking.issued_ns;
      terminal.last_command = previous_tracking.control.torque;
      terminal.requested_position = manager.requested_command().position;
      terminal.requested_pose_valid = manager.requested_command().valid;
      terminal.requested_orientation = manager.requested_command().orientation;
      terminal.reference_position = sample.output.target_position;
      terminal.reference_orientation = sample.output.target_orientation;
      terminal.callback_us = (monotonic_ns() - sample.stamp_ns) * 1e-3;
      terminal.callback_cpu_us = sample.cpu_start_ns ? (thread_cpu_ns() - sample.cpu_start_ns) * 1e-3 : 0.;
      terminal.callback_cpu = sched_getcpu();
      cbf_fault = status;
      if (status == cbf::Status::invalid_model) {
        terminal.control_terminated = true;
        session.failed.store(true, std::memory_order_release);
        throw CbfRejected{status};
      }
      // Latch both arms into the existing bounded braking-and-hold response.
      // The failed filter is bypassed from this point because it cannot certify
      // another command; actuator limits still apply to every protective torque.
      session.protective_stop.store(true, std::memory_order_release);
      manager.request_protective_stop(ft::Reason::cbf_failure);
      const auto stopped = manager.update(sample.state, monotonic_ns(), period);
      sample.homing = stopped.homing;
      sample.output = stopped.control;
      sample.motion_requested = false;
      sample.output.torque = bounds.clamp(sample.output.nominal);
      terminal.protective_command_issued = true;
      terminal.protective_command = sample.output.torque;
      terminal_snapshot_pending.store(true, std::memory_order_release);
    };
    double shutdown_seconds = 0;
    if (cbf_filter && !cbf_shadow &&
        (cbf_filter->configuration().table.enabled ||
         cbf_filter->configuration().self_collision.enabled ||
         cbf_filter->configuration().inter_arm.enabled ||
         cbf_filter->configuration().singularity.enabled))
      manager.reference_projection(
          [](const void* context, const ft::Vector6& step) noexcept -> ft::Vector6 {
            const auto& p = *static_cast<const ProjectionContext*>(context);
            return p.local->reference_normals().project(step);
          },
          &projection_context, true);
    auto step = [&](Telemetry& telemetry, double period) {
      filter_prepared_this_cycle = false;
      const auto cpu_begin = cbf_filter ? telemetry.cpu_start_ns : 0;
      bool outgoing_verified = false;
      bool held_local_correction_verified = false;
      ft::Command c;
      const auto overflow_count = overflows.load(std::memory_order_acquire);
      const bool overflow = overflow_count != seen_overflows;
      seen_overflows = overflow_count;
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
      if (session.protective_stop.load(std::memory_order_acquire))
        manager.request_protective_stop(ft::Reason::cbf_failure);
      // ROS can enqueue input after the RobotState snapshot was timestamped.
      // Evaluate input watchdogs after draining the queues, so newly received
      // input cannot appear to arrive in the future. Keep the original model
      // timestamp for coherent tracking diagnostics.
      const auto input_check_ns = monotonic_ns();

      const auto bounds = local::actuator_bounds(telemetry.state.previous_torque, torque_limits,
                                                 maximum_torque_rate, period);
      if (!bounds)
        throw std::runtime_error("invalid actuator interval");
      if (cbf_filter && count == 2 && session.failed.load(std::memory_order_acquire))
        reject_cbf(cbf::Status::invalid_model, "peer_owner_failure", telemetry, *bounds, period,
                   false);
      cbf::Status preparation_status = cbf::Status::invalid_model;
      auto prepare_filter = [&](bool movable) noexcept {
        const auto& m = telemetry.model;
        if (cbf_filter->configuration().uses_body_volumes)
          finger_opening_exchange.read(finger_opening);
        const cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose,
                                  m.frame_jacobian, telemetry.stamp_ns, finger_opening};
        preparation_status = cbf_filter->prepare(
            view, {bounds->lower, bounds->upper}, period, monotonic_ns(),
            session.collision_exchange[arm] ? &peer_snapshot : nullptr,
            session.collision_exchange[arm] ? &collision_exchange : nullptr, movable);
        filter_prepared_this_cycle = true;
      };
      const auto before_reference = [](void* context, bool movable) noexcept {
        (*static_cast<const decltype(prepare_filter)*>(context))(movable);
      };
      const bool already_protective = session.protective_stop.load(std::memory_order_acquire);
      const auto managed = manager.update(
          telemetry.state, input_check_ns, period,
          cbf_filter && !already_protective ? +before_reference : nullptr,
          cbf_filter && !already_protective ? static_cast<void*>(&prepare_filter) : nullptr);
      telemetry.homing = managed.homing;
      telemetry.output = managed.control;
      telemetry.motion_requested = managed.control.motion_enabled;
      if (cbf_filter && !already_protective &&
          (!filter_prepared_this_cycle ||
           preparation_status != cbf::Status::solved_hard) &&
          !cbf_shadow)
        reject_cbf(preparation_status, "prepare", telemetry, *bounds, period, false);
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
      if (session.protective_stop.load(std::memory_order_acquire)) {
        telemetry.output.torque = bounds->clamp(telemetry.output.nominal);
      } else if (!cbf_filter) {
        // Exact pre-CBF nominal/clipping path; no solver/model/geometry work.
        telemetry.output.torque = bounds->clamp(telemetry.output.nominal);
      } else if (cbf_shadow) {
        // Observation only: no reference projection, holding law, or candidate
        // torque may affect the nominal controller's actuator-bounded output.
        cbf_filter->solve_prepared(telemetry.output.nominal);
        telemetry.output.torque = bounds->clamp(telemetry.output.nominal);
      } else {
        const auto& filtered = cbf_filter->solve_prepared(telemetry.output.nominal);
        if (!filtered.accepted()) {
          reject_cbf(filtered.status, "solve", telemetry, *bounds, period, true);
        }
        if (session.protective_stop.load(std::memory_order_acquire)) {
          // reject_cbf has replaced the nominal law with protective braking.
        } else {
          auto command = bounds->clamp(filtered.torque);
          if (cbf_filter->configuration().inter_arm.enabled &&
              telemetry.output.reason == ft::Reason::position_hold &&
              !ft::preserves_held_arm(bounds->clamp(telemetry.output.nominal), command,
                                      cbf_filter->configuration().hard_tolerance)) {
            // Keep the captured pose/joint anchor. Local safety limiting (for
            // example release braking) is not intentional inter-arm yielding.
            held_local_correction_verified =
                cbf_filter->hold_correction_is_local(telemetry.output.nominal);
            if (!held_local_correction_verified)
              reject_cbf(cbf::Status::constraint_violation, "held_arm_requires_intervention",
                         telemetry, *bounds, period, true);
          }
          if (!session.protective_stop.load(std::memory_order_acquire) &&
              ((command - filtered.torque).cwiseAbs().maxCoeff() >
                   cbf_filter->configuration().hard_tolerance ||
               !cbf_filter->verify_command(command))) {
            reject_cbf(cbf::Status::constraint_violation, "verify", telemetry, *bounds, period,
                       true);
          }
          if (!session.protective_stop.load(std::memory_order_acquire)) {
            telemetry.output.torque = command;
            outgoing_verified = true;
          }
        }
      }
      if (cbf_filter && !session.protective_stop.load(std::memory_order_acquire) &&
          (monotonic_ns() - telemetry.stamp_ns) * 1e-3 >
              cbf_filter->configuration().max_callback_time_us)
        reject_cbf(cbf::Status::timing_overrun, "callback_deadline", telemetry, *bounds, period,
                   true);
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
      if (cbf_filter && !session.protective_stop.load(std::memory_order_acquire)) {
        if (!cbf_diagnostics->emplace([&](CbfDiagnostic& diagnostic) noexcept {
              diagnostic.result = cbf_filter->last_result();
              diagnostic.native_recovery_qp =
                  selected_cbf.recovery_torque_method == cbf::RecoveryTorqueMethod::native_qp;
              diagnostic.table_height_m = selected_cbf.table.height;
              diagnostic.table_clearance_m = selected_cbf.table.clearance;
              diagnostic.motion_enabled = telemetry.output.motion_enabled;
              diagnostic.inter_arm_enabled = selected_cbf.inter_arm.enabled;
              diagnostic.peer_valid = peer_snapshot.valid;
              diagnostic.peer_state_ns = peer_snapshot.timestamp_ns;
              diagnostic.peer_age_us = diagnostic.peer_state_ns &&
                                               monotonic_ns() >= diagnostic.peer_state_ns
                                           ? (monotonic_ns() - diagnostic.peer_state_ns) * 1e-3
                                           : 0.;
              diagnostic.actual_command_verified =
                  outgoing_verified || cbf_filter->verify_command(telemetry.output.torque);
              diagnostic.held_local_correction_verified = held_local_correction_verified;
              diagnostic.nominal_inspection =
                  cbf_filter->inspect_command(bounds->clamp(telemetry.output.nominal));
              diagnostic.model_acquisition_us = telemetry.model_acquisition_us;
              diagnostic.fci = telemetry.fci;
              diagnostic.command = telemetry.output.torque;
              diagnostic.cycle = telemetry.cycle;
              diagnostic.state_ns = telemetry.stamp_ns;
              diagnostic.wake_lateness_us = telemetry.wake_lateness_us;
              diagnostic.observed_period_us = telemetry.observed_period_us;
              diagnostic.cpu_us = (thread_cpu_ns() - cpu_begin) * 1e-3;
              diagnostic.callback_us = (monotonic_ns() - telemetry.stamp_ns) * 1e-3;
            }))
          cbf_drops.fetch_add(1, std::memory_order_relaxed);
      }
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
        ft::UrdfModelOutput evaluated;
        const auto evaluate_mock = [&] {
          if (!dynamics.evaluate(sample.state.q, sample.state.dq,
                                 ft::UrdfPandaModel::standard_flange_to_ee(), evaluated))
            throw std::runtime_error("invalid mock URDF state");
          sample.model = evaluated.state;
          sample.model.previous_torque = sample.state.previous_torque;
          sample.state.mass = sample.model.mass;
          sample.state.coriolis = sample.model.coriolis;
          sample.gravity = evaluated.gravity;
        };
        evaluate_mock();
        if (session.collision_exchange[arm]) {
          sample.stamp_ns = monotonic_ns();
          const auto& m = sample.model;
          const cbf::ModelView view{
              m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, sample.stamp_ns};
          session.collision_exchange[arm]->publish(
              ft::stationary_collision_seed(view, cbf_filter->configuration()));
          connected.fetch_add(1, std::memory_order_acq_rel);
          // Startup barrier only, outside the callback. Never bootstrap with a missing peer.
          while (connected.load(std::memory_order_acquire) < count &&
                 !session.failed.load(std::memory_order_acquire))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          if (session.failed.load(std::memory_order_acquire))
            throw std::runtime_error("peer startup failed");
        }
        MockPriority priority(mock_realtime);
        uint64_t previous_stamp = 0;
        auto deadline = Clock::now();
        while (!(sample.finish_requested && session.may_finish())) {
          sample.stamp_ns = monotonic_ns();
          sample.cpu_start_ns = cbf_filter ? thread_cpu_ns() : 0;
          sample.wake_lateness_us = std::max(
              0., std::chrono::duration<double, std::micro>(Clock::now() - deadline).count());
          sample.observed_period_us =
              previous_stamp ? (sample.stamp_ns - previous_stamp) * 1e-3 : 1000.;
          previous_stamp = sample.stamp_ns;
          evaluate_mock();
          sample.model_acquisition_us = (monotonic_ns() - sample.stamp_ns) * 1e-3;
          step(sample, 0.001);
          sample.state.previous_torque = sample.output.torque;
          deadline += std::chrono::milliseconds(1);
          std::this_thread::sleep_until(deadline);
        }
      } else {
        franka::Robot robot(ip);  // Enforces libfranka realtime requirements.
        const auto startup_state = robot.readOnce();
        const auto startup_stamp = monotonic_ns();
        std::ostringstream startup;
        startup << "FCI startup robot_ip=" << ip << " mode=" << startup_state.robot_mode
                << " current_errors=" << startup_state.current_errors
                << " last_motion_errors=" << startup_state.last_motion_errors;
        RCLCPP_INFO(node->get_logger(), "%s", startup.str().c_str());
        ft::require_idle_control_start(startup_state);
        if (session.collision_exchange[arm]) {
          const auto feedback = ft::franka_model_feedback(startup_state, dynamics);
          const auto& m = feedback.model;
          const cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose,
                                    m.frame_jacobian, startup_stamp};
          const auto seed = ft::stationary_collision_seed(view, cbf_filter->configuration());
          if (!seed.valid)
            throw std::runtime_error("inter-arm startup requires valid stationary arm feedback");
          session.startup_seed_ns[arm].store(seed.timestamp_ns, std::memory_order_release);
          session.collision_exchange[arm]->publish(seed);
        }
        connected.fetch_add(1, std::memory_order_acq_rel);
        while (connected.load(std::memory_order_acquire) < count &&
               !session.failed.load(std::memory_order_acquire))
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (session.failed.load(std::memory_order_acquire))
          throw std::runtime_error("peer startup failed");
        if (session.collision_exchange[arm]) {
          // The first seed may have aged at the connection barrier. Acquire a
          // new state AFTER waiting; no waiting/logging follows publication.
          // Unchanged freshness checks still reject excessive FCI negotiation.
          const auto seed = ft::acquire_stationary_collision_seed(
              [&]() { return robot.readOnce(); },
              []() { return monotonic_ns(); },
              [&](const franka::RobotState& fresh) {
                ft::require_idle_control_start(fresh);
                return ft::franka_model_feedback(fresh, dynamics).model;
              },
              cbf_filter->configuration());
          if (!seed.valid)
            throw std::runtime_error("inter-arm startup refresh requires stationary arm feedback");
          session.startup_seed_ns[arm].store(seed.timestamp_ns, std::memory_order_release);
          session.collision_exchange[arm]->publish(seed);
        }
        uint64_t previous_hardware_stamp = 0;
        const auto torque_callback = [&](const franka::RobotState& state,
                                         franka::Duration period) -> franka::Torques {
          Telemetry sample;
          sample.stamp_ns = monotonic_ns();
          sample.cpu_start_ns = cbf_filter ? thread_cpu_ns() : 0;
          sample.observed_period_us =
              previous_hardware_stamp ? (sample.stamp_ns - previous_hardware_stamp) * 1e-3 : 0.;
          previous_hardware_stamp = sample.stamp_ns;
          sample.native = state;
          const auto timing = robot.controlTiming();
          if (timing.update_start_ns && timing.receive_start_ns >= timing.update_start_ns &&
              timing.receive_end_ns >= timing.receive_start_ns &&
              timing.update_end_ns >= timing.receive_end_ns &&
              sample.stamp_ns >= timing.update_end_ns) {
            sample.fci = {true,
                          timing.waited,
                          state.time.toMSec(),
                          timing.received_states,
                          (timing.update_end_ns - timing.update_start_ns) * 1e-3,
                          timing.update_cpu_ns * 1e-3,
                          (timing.receive_end_ns - timing.receive_start_ns) * 1e-3,
                          (timing.update_end_ns - timing.receive_end_ns) * 1e-3,
                          (sample.stamp_ns - timing.update_end_ns) * 1e-3};
          }
          const auto feedback = ft::franka_model_feedback(state, dynamics);
          sample.state = feedback.control;
          sample.model = feedback.model;
          sample.gravity = feedback.gravity;
          sample.model_acquisition_us = (monotonic_ns() - sample.stamp_ns) * 1e-3;
          step(sample, period.toSec() == 0 ? .001 : period.toSec());
          std::array<double, 7> torque{};
          std::copy_n(sample.output.torque.data(), 7, torque.begin());
          if (sample.finish_requested && session.may_finish())
            return franka::MotionFinished(franka::Torques(torque));
          return franka::Torques(torque);
        };
        control_request_ns = monotonic_ns();
        control_requested.store(true, std::memory_order_release);
        if (session.collision_exchange[arm]) {
          robot.controlWithStartupObserver(
              torque_callback,
              [&](const franka::RobotState& state) {
                // Same owner, same connection: genuinely received startup UDP
                // states bridge the blocking Move-acknowledgement handoff.
                // No command is produced here and no age check is bypassed.
                const auto stamp = monotonic_ns();
                try {
                  if (session.failed.load(std::memory_order_acquire))
                    throw std::runtime_error("peer failed during observed FCI startup");
                  ft::require_startup_observation(state);
                  const auto feedback = ft::franka_model_feedback(state, dynamics);
                  const auto& m = feedback.model;
                  const cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis,
                                            m.frame_pose, m.frame_jacobian, stamp};
                  const auto snapshot = ft::stationary_collision_seed(view, cbf_filter->configuration());
                  if (!snapshot.valid)
                    throw std::runtime_error("invalid/moving arm during observed FCI startup");
                  session.collision_exchange[arm]->publish(snapshot);
                  ++startup_observation_count;
                  last_startup_observation_ns = stamp;
                } catch (...) {
                  // Cancellation may wait for transport; latch the pair first.
                  session.failed.store(true, std::memory_order_release);
                  throw;
                }
              }, true, franka::kMaxCutoffFrequency);
        } else {
          robot.control(torque_callback, true, franka::kMaxCutoffFrequency);
        }
      }
    } catch (...) {
      // Latch before joining publishers or logging; the peer must stop accepting
      // commands as soon as the hardware/control owner has failed.
      session.failed.store(true, std::memory_order_release);
      join_executor();
      if (control_request_ns)
        RCLCPP_ERROR(node->get_logger(),
                     "FCI startup observations=%llu last_state_ns=%llu control_request_ns=%llu",
                     static_cast<unsigned long long>(startup_observation_count),
                     static_cast<unsigned long long>(last_startup_observation_ns),
                     static_cast<unsigned long long>(control_request_ns));
      if (terminal_snapshot && terminal_snapshot->present) {
        // The tee'd launch log is the durable primary capture. A separate ROS
        // event is useful for bags but is not the only copy at process exit.
        std::ostringstream json;
        ft::write_cbf_terminal_snapshot(json, *terminal_snapshot, dynamics.fingerprint());
        const auto payload = json.str();
        RCLCPP_ERROR(node->get_logger(), "CBF_CONTROL_TERMINATED_SNAPSHOT %s", payload.c_str());
        if (rclcpp::ok()) {
          try {
            std_msgs::msg::String message;
            message.data = payload;
            cbf_rejection_publisher->publish(message);
            // A callback-1 fault can precede recorder topic discovery. Keep the
            // transient-local sample alive for late readers after robot.control
            // and executor teardown; no torque ownership or callback is waiting.
            const auto discovery_deadline = Clock::now() + std::chrono::milliseconds(500);
            while (rclcpp::ok() && Clock::now() < discovery_deadline)
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            cbf_rejection_publisher->wait_for_all_acked(std::chrono::milliseconds(250));
          } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(),
                        "CBF terminal ROS event unavailable; launch log retained: %s", e.what());
          }
        }
      }
      throw;
    }
    join_executor();
  } catch (const CbfRejected& fault) {
    RCLCPP_ERROR(
        node->get_logger(),
        "CBF torque rejected: %s; control terminated, no unchecked fallback command issued",
        cbf::name(fault.status));
    session.failed.store(true, std::memory_order_release);
    result = 2;
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
  auto root = std::make_shared<rclcpp::Node>("panda_cartesian_pd_controller");
  int result = 2;
  try {
    const auto mode = parameter<std::string>(root, "mode", "single");
    const auto backend = parameter<std::string>(root, "backend", "mock");
    const bool hardware = backend == "hardware";
    if ((mode != "single" && mode != "dual") || (backend != "mock" && !hardware))
      throw std::invalid_argument("mode must be single/dual and backend mock/hardware");
    std::array<std::shared_ptr<rclcpp::Node>, 2> nodes;
    const int count = mode == "dual" ? 2 : 1;
    std::array<bool, 2> cbf_enabled{};
    std::array<std::string, 2> cbf_modes;
    for (int i = 0; i < count; ++i) {
      const std::string prefix = count == 1 ? "" : (i == 0 ? "left." : "right.");
      cbf_enabled[i] = parameter(root, prefix + "cbf_enabled", false);
      cbf_modes[i] = parameter<std::string>(
          root, prefix + "cbf_commissioning_mode", hardware ? "" : "filtered");
      if (hardware && cbf_enabled[i] && cbf_modes[i].empty())
        throw std::invalid_argument(
            "enabled hardware CBF requires cbf_commissioning_mode shadow or filtered");
      if ((cbf_enabled[i] || !cbf_modes[i].empty()) && cbf_modes[i] != "shadow" &&
          cbf_modes[i] != "filtered")
        throw std::invalid_argument("CBF output mode must be shadow or filtered");
    }
    if (hardware && count == 2 && (cbf_enabled[0] || cbf_enabled[1]) &&
        (!cbf_enabled[0] || !cbf_enabled[1] || cbf_modes[0] != cbf_modes[1]))
      throw std::invalid_argument(
          "dual hardware CBF requires both arms enabled in the same output mode");
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
          {"backend", backend}, {"robot_ip", ip}, {"base_frame", frame},
          {"controller_id", id}, {"homing_enabled", parameter<bool>(root, "homing_enabled", true)}};
      for (const auto& entry : std::vector<std::pair<std::string, std::string>>{
               {"cartesian_control_law", "cartesian_pd"},
               {"dynamics_model_file", kUrdfResource},
               {"command_topic", "cartesian_teleop_command"},
               {"measured_pose_topic", "teleop_anchor_pose"},
               {"state_topic", "robot_state"},
               {"controller_state_topic", "/quest_usb_bridge/" + id + "/controller_state"}})
        overrides.emplace_back(entry.first,
                               parameter<std::string>(root, prefix + entry.first, entry.second));
      overrides.emplace_back("cbf_enabled", cbf_enabled[i]);
      overrides.emplace_back("cbf_config_path",
                             parameter<std::string>(root, prefix + "cbf_config_path", ""));
      overrides.emplace_back("cbf_commissioning_mode", cbf_modes[i]);
      for (const auto* name : {"reference_shaping_enabled", "tracking_protection_enabled",
                               "cartesian_wrench_limits_enabled", "mock_realtime"})
        overrides.emplace_back(name, parameter(root, prefix + name, false));
      for (const auto& entry :
           std::vector<std::pair<std::string, double>>{{"nullspace_stiffness", 10.0},
                                                       {"inverse_dynamics_damping_ratio", 1.0},
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
               {"inverse_dynamics_stiffness",
                {parameter(root, prefix + "lambda_translation", 50.) *
                     parameter(root, prefix + "gain_translation", 20.),
                 parameter(root, prefix + "lambda_translation", 50.) *
                     parameter(root, prefix + "gain_translation", 20.),
                 parameter(root, prefix + "lambda_translation", 50.) *
                     parameter(root, prefix + "gain_translation", 20.),
                 parameter(root, prefix + "lambda_rotation", 40.) *
                     parameter(root, prefix + "gain_rotation", 2.5),
                 parameter(root, prefix + "lambda_rotation", 40.) *
                     parameter(root, prefix + "gain_rotation", 2.5),
                 parameter(root, prefix + "lambda_rotation", 40.) *
                     parameter(root, prefix + "gain_rotation", 2.5)}},
               {"inverse_dynamics_nullspace_stiffness", std::vector<double>(7, 10.)},
               {"inverse_dynamics_nullspace_damping", std::vector<double>(7, 6.324555320336759)},
               {"home_joint_position",
                std::vector<double>(local::kHome.begin(), local::kHome.end())},
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
                           "panda_cartesian_pd_controller", ns,
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
    if (count == 2) {
      std::array<cbf::Configuration, 2> configurations;
      for (int arm = 0; arm < 2; ++arm) {
        if (!cbf_enabled[arm])
          continue;
        const std::string prefix = arm == 0 ? "left." : "right.";
        configurations[arm] = cbf::load_configuration(
            parameter<std::string>(root, prefix + "cbf_config_path", ""),
            arm == 0 ? "left" : "right");
      }
      if ((configurations[0].inter_arm.enabled || configurations[1].inter_arm.enabled) &&
          (!cbf_enabled[0] || !cbf_enabled[1] || !configurations[0].inter_arm.enabled ||
           !configurations[1].inter_arm.enabled))
        throw std::invalid_argument("inter-arm collision requires both CBF owners and profiles");
      if (configurations[0].inter_arm.enabled &&
          configurations[0].uses_body_volumes != configurations[1].uses_body_volumes)
        throw std::invalid_argument(
            "inter-arm profiles must use the same collision geometry format");
    }
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
