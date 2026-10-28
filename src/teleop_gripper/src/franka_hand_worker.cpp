#include <franka/exception.h>
#include <franka/gripper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <quest_teleop_interfaces/msg/gripper_status.hpp>
#include <quest_teleop_interfaces/msg/quest_controller_state.hpp>
#include <quest_teleop_interfaces/srv/calibrate_gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <thread>

#include "hand_reporting.hpp"
#include "teleop_gripper/hand_core.hpp"
#include "teleop_gripper/hand_worker.hpp"

namespace ft = teleop_gripper;
using Clock = std::chrono::steady_clock;
uint64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
}
const char* operation_name(ft::HandOperation operation) {
  switch (operation) {
    case ft::HandOperation::calibrate:
      return "calibrate";
    case ft::HandOperation::open:
      return "open";
    case ft::HandOperation::close:
      return "close";
    case ft::HandOperation::stop:
      return "stop";
    default:
      return "idle";
  }
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("franka_hand_worker");
  int exit_code = 0;
  try {
    const auto backend = node->declare_parameter<std::string>("backend", "mock");
    const auto ip = node->declare_parameter<std::string>("robot_ip", "");
    const auto confirmed = node->declare_parameter<bool>("operator_confirm", false);
    const auto calibrate_on_start = node->declare_parameter<bool>("calibrate_on_start", false);
    const auto controller = node->declare_parameter<std::string>("controller_id", "right");
    const auto topic = node->declare_parameter<std::string>(
        "controller_state_topic", "/quest_usb_bridge/" + controller + "/controller_state");
    if ((backend != "mock" && backend != "hardware") ||
        (controller != "right" && controller != "left"))
      throw std::invalid_argument("invalid backend/controller ID");
    const bool hardware = backend == "hardware";
    if (hardware && (!confirmed || ip.empty()))
      throw std::invalid_argument("hardware Hand requires robot_ip and operator_confirm:=true");
    ft::HandParameters p;
    p.speed = node->declare_parameter("speed_mps", p.speed);
    p.open_speed = node->declare_parameter("open_speed_mps", p.open_speed);
    p.close_width = node->declare_parameter("close_width_m", p.close_width);
    p.grasp_force = node->declare_parameter("grasp_force_n", p.grasp_force);
    p.cable_diameter = node->declare_parameter("cable_diameter_m", p.cable_diameter);
    p.cable_tolerance = node->declare_parameter("cable_diameter_tolerance_m", p.cable_tolerance);
    p.open_width = node->declare_parameter("open_width_m", p.open_width);
    p.engage = node->declare_parameter("joystick_engage", p.engage);
    p.neutral = node->declare_parameter("joystick_neutral", p.neutral);
    // Old profiles may still contain retired side-grip/click/retry settings.
    for (const auto* legacy : {"grasp_width_m", "epsilon_inner_m", "epsilon_outer_m", "grip_press",
                               "grip_release", "grip_debounce_ms", "close_retry_interval_ms",
                               "grip_click_max_ms", "grip_double_click_ms"}) {
      node->declare_parameter<double>(legacy, -1.0);
      if (node->get_node_parameters_interface()->get_parameter_overrides().count(legacy))
        RCLCPP_WARN(node->get_logger(), "Retired Hand setting %s is ignored by joystick control",
                    legacy);
    }
    const double poll = node->declare_parameter("worker_poll_ms", 1.0);
    const double timeout = node->declare_parameter("command_timeout_ms", 100.0);
    const double uncertainty = node->declare_parameter("max_clock_uncertainty_ms", 10.0);
    const double mock_duration = node->declare_parameter("mock_operation_duration_ms", 100.0);
    const double mock_object = node->declare_parameter("mock_object_width_m", 0.0);
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 100 || !std::isfinite(uncertainty) ||
        uncertainty <= 0 || uncertainty > 10 || !std::isfinite(mock_duration) ||
        mock_duration < 0 || mock_duration > 10000 || !std::isfinite(mock_object) ||
        mock_object < 0 || mock_object > .08)
      throw std::invalid_argument("invalid Hand freshness/mock settings");
    if (!std::isfinite(poll) || poll < 1 || poll > 20)
      throw std::invalid_argument("Hand worker_poll_ms must be 1..20 ms");
    p.timeout_ns = static_cast<uint64_t>(timeout * 1e6);
    p.uncertainty_ns = static_cast<uint64_t>(uncertainty * 1e6);
    if (!p.valid())
      throw std::invalid_argument("invalid Hand settings");
    RCLCPP_INFO(node->get_logger(),
                "Hand joystick control: inward closes at %.6g m/s, outward opens at %.6g m/s; "
                "neutral cancels unfinished motion and preserves a confirmed force grasp. "
                "Cable diameter=%.6g +/- %.6g m, grasp_force_n=%.6g; worker poll=%.6g ms.",
                p.speed, p.open_speed, p.cable_diameter, p.cable_tolerance, p.grasp_force, poll);
    if (p.grasp_force < 0)
      RCLCPP_WARN(node->get_logger(),
                  "Hand closing requires grasp_force_n to be configured for the cable; opening "
                  "remains available");
    ft::HandCore core(p);
    ft::HandWorker worker(hardware, ip, mock_duration, mock_object);
    std::mutex control_mutex;
    RCLCPP_INFO(node->get_logger(), "Hand backend=%s robot_ip=%s controller=%s input_topic=%s",
                backend.c_str(), ip.c_str(), controller.c_str(), topic.c_str());
    bool input_seen = false;
    uint64_t last_input_log_ns = 0;
    std::array<bool, 8> last_input_flags{};
    ft::HandOperation operation = ft::HandOperation::none;
    std::string fault_detail;
    uint64_t started = 0, completed = 0;
    auto events = node->create_publisher<quest_teleop_interfaces::msg::GripperStatus>(
        "gripper_events", rclcpp::QoS(1).transient_local());
    ft::HandReporting reporting(events);
    auto publish = [&](const std::string& detail = "") {
      quest_teleop_interfaces::msg::GripperStatus status;
      status.header.stamp = node->now();
      status.state =
          core.state() == ft::HandState::holding
              ? "holding"
              : (core.state() == ft::HandState::open
                     ? "open"
                     : (core.state() == ft::HandState::stopped
                            ? "stopped"
                            : (core.state() == ft::HandState::closed ? "closed" : "unknown")));
      status.neutral_required = core.release_required();
      status.grasp_confirmed = core.grasp_confirmed();
      status.commanded_grasp_force_n = core.commanded_force();
      status.operation = operation_name(operation);
      status.message = detail.empty() ? core.message() : detail;
      if (!fault_detail.empty())
        status.message += "; " + fault_detail;
      status.calibrated = core.calibrated();
      status.busy = core.busy();
      status.hardware_backend = hardware;
      status.width = core.width();
      status.maximum_width = core.maximum_width();
      status.operations_started = started;
      status.operations_completed = completed;
      reporting.status(std::move(status));
    };
    auto issue = [&](const ft::HandRequest& request) {
      if (request.operation == ft::HandOperation::none)
        return;
      operation = request.operation;
      if (operation == ft::HandOperation::calibrate)
        fault_detail.clear();
      ++started;
      const auto now = monotonic_ns(), source = core.input_source_ns();
      const double age = source && now >= source ? (now - source) / 1e6 : -1;
      worker.issue(request, core.width());
      reporting.log([node, request, age, now] {
        RCLCPP_INFO(
            node->get_logger(),
            "Hand request: operation=%s width_m=%.9g speed_m_s=%.9g force_n=%.9g input_age_ms=%.3f "
            "dispatch_pc_monotonic_ns=%llu",
            operation_name(request.operation), request.width, request.speed, request.force, age,
            static_cast<unsigned long long>(now));
      });
      publish();
    };
    auto calibration = node->create_service<quest_teleop_interfaces::srv::CalibrateGripper>(
        "calibrate_gripper",
        [&](const std::shared_ptr<quest_teleop_interfaces::srv::CalibrateGripper::Request> request,
            std::shared_ptr<quest_teleop_interfaces::srv::CalibrateGripper::Response> response) {
          std::lock_guard<std::mutex> lock(control_mutex);
          const auto action =
              core.calibrate(request->empty_hand_confirm, request->operator_confirm);
          response->accepted = action.operation == ft::HandOperation::calibrate;
          response->message = core.message();
          issue(action);
          publish();
        });
    auto input = node->create_subscription<quest_teleop_interfaces::msg::QuestControllerState>(
        topic, rclcpp::QoS(1).best_effort(),
        [&](quest_teleop_interfaces::msg::QuestControllerState::ConstSharedPtr state) {
          if (state->controller_id != controller)
            return;
          std::lock_guard<std::mutex> lock(control_mutex);
          ft::GripInput c;
          c.sequence = state->sequence;
          c.source_ns = state->source_time_pc_monotonic_ns;
          c.uncertainty_ns = state->clock_uncertainty_ns;
          c.generation = state->reference_space_generation;
          c.value = ft::inward_joystick(state->thumbstick_x, controller == "right");
          c.joystick_active = state->thumbstick_active;
          // Finger control depends on the available joystick, not Cartesian
          // tracking, trigger, side grip, or thumbstick click.
          c.valid = state->connection_valid && state->session_focused && state->source_time_valid &&
                    state->clock_sync_valid && state->reference_space == "LOCAL" &&
                    !state->session_id.empty() && state->session_id.size() <= 64 &&
                    state->session_id.find('\0') == std::string::npos;
          if (c.valid)
            std::copy(state->session_id.begin(), state->session_id.end(), c.session.begin());
          const auto previous_message = core.message();
          const bool previous_neutral_required = core.release_required();
          const auto now = monotonic_ns();
          issue(core.input(c, now));
          const std::array<bool, 8> flags{
              core.calibrated(),      core.release_required(), c.valid,
              state->session_focused, c.joystick_active,       std::abs(c.value) <= p.neutral,
              c.value >= p.engage,    c.value <= -p.engage};
          if ((!input_seen || flags != last_input_flags) && now - last_input_log_ns >= 500000000) {
            input_seen = true;
            last_input_log_ns = now;
            last_input_flags = flags;
            reporting.log([node, c, flags, now, message = std::string(core.message())] {
              RCLCPP_INFO(node->get_logger(),
                          "Hand input: calibrated=%d neutral_required=%d valid=%d focused=%d "
                          "joystick_active=%d inward_axis=%.3f input_age_ms=%.3f detail=%s",
                          flags[0], flags[1], c.valid, flags[3], c.joystick_active, c.value,
                          c.source_ns && now >= c.source_ns ? (now - c.source_ns) / 1e6 : -1.,
                          message.c_str());
            });
          }
          if (previous_message != core.message() ||
              previous_neutral_required != core.release_required())
            publish();
        });
    auto poll_worker = [&] {
      const auto previous_message = core.message();
      issue(core.watchdog(monotonic_ns()));
      if (previous_message != core.message())
        publish();
      if (const auto result = worker.collect()) {
        const auto next = core.complete(
            result->operation, result->success, result->communication_ok, result->width,
            result->maximum_width, monotonic_ns(), result->grasp_confirmed);
        if (!core.calibrated())
          fault_detail = result->error.empty()
                             ? "Operation failed or width readback was invalid; inspect Hand result"
                             : result->error;
        operation = ft::HandOperation::none;
        completed += result->stopped ? 2 : 1;
        // Dispatch the latest held direction before diagnostic formatting or DDS work.
        issue(next);
        reporting.log([node, result = *result, calibrated = core.calibrated(),
                       collected_ns = monotonic_ns(), message = std::string(core.message()),
                       recovery_service = std::string(calibration->get_service_name()),
                       maximum = core.maximum_width(), ip, controller, p] {
          RCLCPP_INFO(node->get_logger(),
                      "Hand result: operation=%s success=%d communication_ok=%d stopped=%d "
                      "width_m=%.9g maximum_width_m=%.9g endpoint_excess_um=%.3f "
                      "duration_ms=%.3f command_ms=%.3f readback_ms=%.3f "
                      "stop_completion_ms=%.3f stop_call_ms=%.3f stop_calls=%u "
                      "stop_acknowledged=%d stop_retry_exhausted=%d "
                      "collected_pc_monotonic_ns=%llu grasp_confirmed=%d error=%s",
                      operation_name(result.operation), result.success, result.communication_ok,
                      result.stopped, result.width, result.maximum_width,
                      std::max({0., -result.width, result.width - result.maximum_width}) * 1e6,
                      result.duration_ms, result.command_ms, result.readback_ms,
                      result.stop_duration_ms, result.stop_call_ms, result.stop_calls,
                      result.stop_acknowledged, result.stop_retry_exhausted,
                      static_cast<unsigned long long>(collected_ns), result.grasp_confirmed,
                      result.error.c_str());
          if (!calibrated)
            RCLCPP_ERROR(node->get_logger(),
                         "Hand operation did not leave a calibrated worker: %s. "
                         "Opening/closing disabled. Backend: %s. After the worker is idle and "
                         "the fingers are empty, explicitly call %s with both confirmations.",
                         message.c_str(), result.error.c_str(), recovery_service.c_str());
          if (result.operation == ft::HandOperation::calibrate && calibrated)
            RCLCPP_INFO(
                node->get_logger(),
                "Hand calibration ready: robot_ip=%s maximum_gap_m=%.6f. "
                "Center the %s joystick within +/- %.3f, then deflect inward past %.3f to close.",
                ip.c_str(), maximum, controller.c_str(), p.neutral, p.engage);
        });
        publish();
      }
    };
    publish();
    if (calibrate_on_start) {
      RCLCPP_INFO(node->get_logger(), "Starting empty-hand calibration requested by launch");
      issue(core.calibrate(true, !hardware || confirmed));
    } else {
      RCLCPP_INFO(node->get_logger(), "%s Hand worker; manual empty-hand calibration required",
                  backend.c_str());
    }
    // A continuously ready 1 ms ROS timer can compete with subscription
    // callbacks. Poll completions independently; the core lock never covers
    // SDK calls, DDS publication or console I/O.
    std::atomic<bool> polling{true};
    std::exception_ptr polling_failure;
    std::thread poll_thread([&] {
      try {
        while (polling.load() && rclcpp::ok()) {
          {
            std::lock_guard<std::mutex> lock(control_mutex);
            poll_worker();
          }
          std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(poll * 1e6)));
        }
      } catch (...) {
        polling_failure = std::current_exception();
        rclcpp::shutdown();
      }
    });
    try {
      rclcpp::spin(node);
    } catch (...) {
      polling.store(false);
      poll_thread.join();
      throw;
    }
    polling.store(false);
    poll_thread.join();
    issue(core.shutdown());
    if (polling_failure)
      std::rethrow_exception(polling_failure);
    // Worker destruction stops and joins outstanding motion/cancellation before disconnecting.
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Hand worker stopped: %s", e.what());
    exit_code = 2;
  }
  rclcpp::shutdown();
  return exit_code;
}
