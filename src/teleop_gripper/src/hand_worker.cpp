#include "teleop_gripper/hand_worker.hpp"

#include <franka/exception.h>
#include <franka/gripper.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace teleop_gripper {
using Clock = std::chrono::steady_clock;
namespace {
class FrankaHand final : public HandDevice {
 public:
  explicit FrankaHand(const std::string& ip) : hand_(ip) {}
  bool homing() override { return hand_.homing(); }
  bool move(double width, double speed) override { return hand_.move(width, speed); }
  bool grasp(double width, double speed, double force, double inner, double outer) override {
    return hand_.grasp(width, speed, force, inner, outer);
  }
  bool stop() override { return hand_.stop(); }
  HandFeedback read() override {
    const auto s = hand_.readOnce();
    return {s.width, s.max_width, s.is_grasped};
  }

 private:
  franka::Gripper hand_;
};
}  // namespace
HandWorker::HandWorker(bool hardware, std::string ip, double duration, double object_width)
    : hardware_(hardware),
      ip_(std::move(ip)),
      mock_duration_ms_(duration),
      mock_object_width_(object_width) {}
HandWorker::HandWorker(std::shared_ptr<HandDevice> device)
    : hardware_(true), mock_duration_ms_(0), hand_(std::move(device)) {
  if (!hand_)
    throw std::invalid_argument("Hand device is required");
}
HandWorker::~HandWorker() {
  stop();
  if (motion_.valid())
    motion_.wait();
  if (stop_.valid())
    stop_.wait();
}
void HandWorker::stop() {
  if (!motion_.valid() || stop_.valid())
    return;
  cancel_.store(true);
  stop_started_ = Clock::now();
  stop_call_ms_ = 0;
  stop_calls_ = 0;
  stop_ = std::async(std::launch::async, [this] {
    StopResult result;
    if (!hardware_) {
      result.success = true;
      result.interrupted = !command_done_.load();
      return result;
    }
    const auto completed = [&] {
      result.success = result.acknowledged || stop_calls_ == 0;
      return result;
    };
    // A stop can overtake the first move request. Retry only while the command
    // itself is outstanding, never while final UDP readback is being collected.
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    do {
      if (command_done_.load())
        return completed();  // Already idle; no firmware stop needed.
      if (!command_submitted_.load()) {
        // Input cancellation during live readback or release of an existing
        // grasp prevents the next motion from being submitted. There is no
        // unfinished motion to stop yet, and another stop could release force
        // redundantly or race the release already in progress.
        std::unique_lock<std::mutex> lock(completion_mutex_);
        command_finished_.wait_for(lock, std::chrono::milliseconds(20), [this] {
          return command_done_.load() || command_submitted_.load();
        });
        if (command_done_.load())
          return completed();
        if (!command_submitted_.load())
          continue;
      }
      std::shared_ptr<HandDevice> hand;
      {
        std::lock_guard<std::mutex> lock(connection_mutex_);
        hand = hand_;
      }
      if (!hand) {
        result.error = "Hand stop has no connected device";
        return result;
      }
      {
        // Serialize cancellation with command completion. Once a grasp command
        // has completed, neutral/input loss must not send stop() and release it.
        std::unique_lock<std::mutex> completion_lock(completion_mutex_);
        if (command_done_.load())
          return completed();
        const auto begin = Clock::now();
        ++stop_calls_;
        stop_called_.store(true);
        try {
          if (hand->stop())
            result.acknowledged = true;
          else if (!result.acknowledged)
            result.error = "Hand stop returned unsuccessful";
        } catch (const franka::CommandException& e) {
          // A redundant stop may be rejected after a successful stop. Preserve
          // that acknowledgement, but do not retry an initial rejection.
          if (!result.acknowledged)
            result.error = std::string("Hand stop command rejected: ") + e.what();
        } catch (const std::exception& e) {
          // A transport/protocol/unknown fault must not be hidden by an older
          // successful acknowledgement or retried automatically.
          result.error = std::string("Hand stop device/communication error: ") + e.what();
        } catch (...) {
          result.error = "Hand stop raised an unknown exception";
        }
        stop_call_ms_ += std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
      }
      if (!result.error.empty())
        return result;
      if (command_done_.load())
        return completed();
      // Give command completion priority over redundant stop traffic. This
      // wait wakes immediately when the blocking command returns.
      std::unique_lock<std::mutex> lock(completion_mutex_);
      if (command_finished_.wait_for(lock, std::chrono::milliseconds(20),
                                     [this] { return command_done_.load(); }))
        return completed();
    } while (Clock::now() < deadline);
    // The two-second window bounds retries, not a blocking SDK call. A late
    // successful acknowledgement followed by command completion is valid;
    // never discard it merely because the retry deadline elapsed meanwhile.
    if (command_done_.load())
      return completed();
    result.retry_exhausted = true;
    result.error = "Hand stop retry window elapsed before command completion";
    return result;
  });
}
void HandWorker::issue(const HandRequest& request, double initial_width) {
  if (request.operation == HandOperation::none)
    return;
  if (request.operation == HandOperation::stop) {
    stop();
    return;
  }
  if (motion_.valid() || stop_.valid())
    throw std::logic_error("overlapping Hand motions");
  cancel_.store(false);
  stop_called_.store(false);
  command_submitted_.store(false);
  command_done_.store(false);
  motion_ = std::async(std::launch::async, [this, request, initial_width, begin = Clock::now()] {
    HandResult result;
    result.operation = request.operation;
    std::shared_ptr<HandDevice> hand;
    bool command_error = false;
    const auto command_finished = [&] {
      {
        std::lock_guard<std::mutex> lock(completion_mutex_);
        command_done_.store(true);
      }
      command_finished_.notify_all();
    };
    const auto read = [&] {
      const auto begin = Clock::now();
      try {
        const auto state = hand->read();
        result.readback_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        return state;
      } catch (...) {
        result.readback_ms +=
            std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        throw;
      }
    };
    try {
      if (!hardware_) {
        const auto begin = Clock::now();
        double elapsed = 0;
        const bool calibration = request.operation == HandOperation::calibrate;
        const double target = request.operation == HandOperation::close
                                  ? std::max(request.width, mock_object_width_)
                                  : request.width;
        const double duration = calibration ? mock_duration_ms_ / 1000.
                                            : std::abs(target - initial_width) / request.speed;
        while (!cancel_.load() && elapsed < duration) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
        }
        const double distance = std::min(std::abs(target - initial_width), request.speed * elapsed);
        result.width =
            calibration ? .08 : initial_width + (target >= initial_width ? distance : -distance);
        result.maximum_width = .08;
        result.success = !cancel_.load();
        result.grasp_confirmed = result.success && request.operation == HandOperation::close &&
                                 mock_object_width_ > 0 && request.force > 0 &&
                                 result.width >= request.minimum_grasp_width &&
                                 result.width <= request.maximum_grasp_width;
        if (request.operation == HandOperation::close && !result.grasp_confirmed && !cancel_.load())
          result.success = false;
        result.communication_ok = true;
        command_finished();
      } else {
        {
          std::lock_guard<std::mutex> lock(connection_mutex_);
          hand = hand_;
        }
        if (!hand) {
          if (request.operation != HandOperation::calibrate)
            throw std::runtime_error("explicit calibration required to connect Hand");
          hand = std::make_shared<FrankaHand>(ip_);
          {
            std::lock_guard<std::mutex> lock(connection_mutex_);
            hand_ = hand;
          }
        }
        const auto release_force = [&](const char* context) {
          try {
            if (!hand->stop())
              throw std::runtime_error("stop returned unsuccessful");
          } catch (const std::exception& e) {
            // A failed force-release stop is a device fault, even if the SDK
            // classifies its rejection as a command exception.
            throw std::runtime_error(std::string(context) + ": " + e.what());
          }
        };
        if (holding_ && request.operation != HandOperation::close) {
          release_force("Hand failed to release grasp before opening/calibration");
          holding_ = false;
        }
        double target = request.width;
        bool move_needed = true;
        bool grasp_issued = false;
        if (request.operation == HandOperation::close) {
          const auto current = read();
          if (!valid_hand_feedback(current.width, current.maximum_width))
            throw std::runtime_error("invalid live Hand width");
          target = std::min(target, std::clamp(current.width, 0., current.maximum_width));
          move_needed = current.width > target + 1e-6;
        }
        bool execute = false;
        {
          std::lock_guard<std::mutex> lock(completion_mutex_);
          execute = !cancel_.load() && (request.operation != HandOperation::close || move_needed);
          if (execute)
            command_submitted_.store(true);
        }
        command_finished_.notify_all();
        if (execute) {
          const auto begin = Clock::now();
          try {
            if (request.operation == HandOperation::calibrate) {
              result.success = hand->homing();
            } else if (request.operation == HandOperation::close) {
              if (move_needed) {
                grasp_issued = true;
                result.success = hand->grasp(target, request.speed, request.force,
                                             request.epsilon_inner, request.epsilon_outer);
              }
            } else {
              result.success = hand->move(target, request.speed);
            }
          } catch (...) {
            result.command_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
            throw;
          }
          result.command_ms =
              std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        }
        command_finished();
        const auto state = read();
        result.width = state.width;
        result.maximum_width = state.maximum_width;
        result.grasp_confirmed = request.operation == HandOperation::close && result.success &&
                                 state.is_grasped && state.width >= request.minimum_grasp_width &&
                                 state.width <= request.maximum_grasp_width;
        if (request.operation == HandOperation::close && !result.grasp_confirmed) {
          // A wide SDK success window must not accept empty closure or a cable
          // outside the configured band. Do not leave an unconfirmed force on.
          if (grasp_issued && !stop_called_.load())
            release_force("Hand failed to stop an unconfirmed grasp");
          result.success = false;
        }
        result.communication_ok = true;
      }
    } catch (const franka::CommandException& e) {
      command_finished();
      // Command failures do not authorize automatic closing retries.
      result.error = e.what();
      command_error = true;
    } catch (const std::exception& e) {
      command_finished();
      result.error = e.what();
    }
    if (!result.error.empty()) {
      // A stop commonly aborts the blocking move. Keep a valid readback even
      // when the command throws; stop acknowledgement is checked separately.
      if (hand) {
        try {
          const auto state = read();
          result.width = state.width;
          result.maximum_width = state.maximum_width;
          // A state read alone cannot clear a transport/protocol exception.
          result.communication_ok = command_error;
        } catch (...) {
        }
      }
    }
    result.duration_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    return result;
  });
}
std::optional<HandResult> HandWorker::collect() {
  if (!motion_.valid() || motion_.wait_for(std::chrono::seconds(0)) != std::future_status::ready ||
      (stop_.valid() && stop_.wait_for(std::chrono::seconds(0)) != std::future_status::ready))
    return {};
  auto result = motion_.get();
  if (stop_.valid()) {
    result.stop_duration_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - stop_started_).count();
    result.stopped = true;
    const auto stop = stop_.get();
    result.stop_call_ms = stop_call_ms_;
    result.stop_calls = stop_calls_;
    if (stop_calls_ > 0 || stop.interrupted)
      result.grasp_confirmed = false;
    result.stop_acknowledged = stop.acknowledged;
    result.stop_retry_exhausted = stop.retry_exhausted;
    result.success = stop.success && result.communication_ok;
    if (!stop.success) {
      result.communication_ok = false;
      if (!result.error.empty())
        result.error += "; ";
      result.error += stop.error + "; explicit calibration required";
    }
  }
  holding_ = result.communication_ok && result.grasp_confirmed;
  if (!result.communication_ok) {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    hand_.reset();
  }
  return result;
}
}  // namespace teleop_gripper
