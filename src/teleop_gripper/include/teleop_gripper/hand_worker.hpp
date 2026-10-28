#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "teleop_gripper/hand_core.hpp"

namespace teleop_gripper {
struct HandFeedback {
  double width{0}, maximum_width{0};
  bool is_grasped{false};
};
// Device seam permits deterministic driver races/failures to be tested without
// connecting to a physical Hand. Implementations support concurrent stop().
class HandDevice {
 public:
  virtual ~HandDevice() = default;
  virtual bool homing() = 0;
  virtual bool move(double width, double speed) = 0;
  virtual bool grasp(double width, double speed, double force, double epsilon_inner,
                     double epsilon_outer) = 0;
  virtual bool stop() = 0;
  virtual HandFeedback read() = 0;
};
struct HandResult {
  HandOperation operation{HandOperation::none};
  bool success{false}, communication_ok{false}, stopped{false};
  bool grasp_confirmed{false};
  double duration_ms{0}, stop_duration_ms{0};
  double command_ms{0}, readback_ms{0}, stop_call_ms{0};
  unsigned stop_calls{0};
  bool stop_acknowledged{false}, stop_retry_exhausted{false};
  double width{0}, maximum_width{0};
  std::string error;
};
// Blocking motion and cancellation run independently. Only one motion is
// permitted; the next starts after both its command and stop have completed.
class HandWorker {
 public:
  HandWorker(bool hardware, std::string ip, double mock_duration_ms, double mock_object_width = 0);
  explicit HandWorker(std::shared_ptr<HandDevice> device);
  ~HandWorker();
  void issue(const HandRequest& request, double initial_width);
  std::optional<HandResult> collect();

 private:
  struct StopResult {
    bool success{false}, acknowledged{false}, retry_exhausted{false}, interrupted{false};
    std::string error;
  };
  void stop();
  bool hardware_;
  std::string ip_;
  double mock_duration_ms_;
  double mock_object_width_{0};
  bool holding_{false};
  std::mutex connection_mutex_;
  std::shared_ptr<HandDevice> hand_;
  std::future<HandResult> motion_;
  std::future<StopResult> stop_;
  std::condition_variable command_finished_;
  std::mutex completion_mutex_;
  std::chrono::steady_clock::time_point stop_started_;
  std::atomic<bool> cancel_{false}, command_done_{true};
  std::atomic<bool> stop_called_{false};
  std::atomic<bool> command_submitted_{false};
  double stop_call_ms_{0};
  unsigned stop_calls_{0};
};
}  // namespace teleop_gripper
