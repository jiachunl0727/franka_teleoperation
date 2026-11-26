// Loopback-only regression: delayed Move acknowledgement must not hide fresh
// startup UDP states from the peer snapshot publisher. Never opens a real robot.
#include <dlfcn.h>
#include <gtest/gtest.h>
#include <robot_impl.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "cbf_safety_filter/constraints.hpp"
#include "mock_server.h"
#include "teleop_controllers/inter_arm_policy.hpp"
#include "teleop_controllers/panda_limits.hpp"
#include "teleop_controllers/panda_model_fixture.hpp"

namespace ft = teleop_controllers;
namespace cbf = cbf_safety_filter;
namespace ri = research_interface::robot;
struct StartupRobot : franka::Robot {
  using Impl = franka::Robot::Impl;
};

namespace {
// Delay only this test's receiving thread after the blocking syscall has
// obtained a state. This models a deschedule while newer UDP states queue.
thread_local bool delay_state_receive = false;
std::atomic<bool> receive_delayed{false}, receive_batch_sent{false};
}  // namespace
extern "C" ssize_t recvfrom(int fd, void* buffer, size_t size, int flags, sockaddr* address,
                            socklen_t* address_size) {
  using Receive = ssize_t (*)(int, void*, size_t, int, sockaddr*, socklen_t*);
  static const auto original = reinterpret_cast<Receive>(dlsym(RTLD_NEXT, "recvfrom"));
  const auto result = original(fd, buffer, size, flags, address, address_size);
  if (delay_state_receive && result == sizeof(ri::RobotState)) {
    delay_state_receive = false;
    receive_delayed.store(true, std::memory_order_release);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (!receive_batch_sent.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::yield();
  }
  return result;
}

TEST(InterArmTransport, UsesNewestQueuedStateAfterBlockingReceiveWasDelayed) {
  RobotMockServer server;
  StartupRobot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", ri::kCommandPort), 0);
  receive_delayed = receive_batch_sent = false;
  server
      .generic([&](RobotMockServer::Socket&, RobotMockServer::Socket& udp) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        ri::RobotState state{};
        state.message_id = 500;
        udp.sendBytes(&state, sizeof(state));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        while (!receive_delayed.load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline)
          std::this_thread::yield();
        for (auto id : {503u, 502u, 507u, 506u}) {
          state.message_id = id;
          udp.sendBytes(&state, sizeof(state));
        }
        receive_batch_sent.store(true, std::memory_order_release);
      })
      .spinOnce();
  delay_state_receive = true;
  const auto state = robot.update(nullptr, nullptr);
  delay_state_receive = false;
  ASSERT_TRUE(receive_delayed.load());
  EXPECT_EQ(state.time.toMSec(), 507u);
  const auto timing = robot.controlTiming();
  EXPECT_EQ(timing.received_states, 5u);
  EXPECT_TRUE(timing.waited);
  EXPECT_LT(timing.update_start_ns, timing.receive_end_ns);
  EXPECT_LE(timing.receive_end_ns, timing.update_end_ns);
}

TEST(InterArmTransport, ExcessiveQueuedBurstFailsInsteadOfStarvingCallbackEntry) {
  RobotMockServer server;
  StartupRobot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", ri::kCommandPort), 0);
  std::atomic<bool> sent{false};
  server
      .generic([&](RobotMockServer::Socket&, RobotMockServer::Socket& udp) {
        for (unsigned i = 0; i < 80; ++i) {
          ri::RobotState state{};
          state.message_id = 100 + i;
          udp.sendBytes(&state, sizeof(state));
        }
        sent.store(true, std::memory_order_release);
      })
      .spinOnce();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (!sent.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();
  ASSERT_TRUE(sent.load());
  EXPECT_THROW(robot.update(nullptr, nullptr), franka::NetworkException);
}

TEST(InterArmStartupTransport, PublishesNewStatesDuringDelayedMoveAcknowledgement) {
  RobotMockServer server;
  StartupRobot::Impl robot(std::make_unique<franka::Network>("127.0.0.1", ri::kCommandPort), 0);
  std::atomic<unsigned> sent{0}, last_observed{0};
  server
      .generic([&](RobotMockServer::Socket& tcp, RobotMockServer::Socket& udp) {
        ri::Move::Header request;
        server.receiveRequest<ri::Move>(tcp, &request);
        for (unsigned i = 0; i < 8; ++i) {
          ri::RobotState state{};
          state.message_id = 100 + i;
          state.q[0] = .01 * (i + 1);
          state.motion_generator_mode = ri::MotionGeneratorMode::kIdle;
          state.controller_mode = ri::ControllerMode::kOther;
          state.robot_mode = ri::RobotMode::kIdle;
          udp.sendBytes(&state, sizeof(state));
          ++sent;
          const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
          while (last_observed.load(std::memory_order_acquire) < state.message_id &&
                 std::chrono::steady_clock::now() < until)
            std::this_thread::yield();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        server.sendResponse<ri::Move>(
            tcp,
            ri::Move::Header(ri::Move::kCommand, request.command_id,
                             sizeof(ri::Move::Message<ri::Move::Response>)),
            ri::Move::Response(ri::Move::Status::kMotionStarted));
        ri::RobotState moving{};
        moving.message_id = 108;
        moving.q[0] = .09;
        moving.motion_generator_mode = ri::MotionGeneratorMode::kJointVelocity;
        moving.controller_mode = ri::ControllerMode::kExternalController;
        moving.robot_mode = ri::RobotMode::kMove;
        udp.sendBytes(&moving, sizeof(moving));
      })
      .spinOnce();

  std::vector<double> positions;
  unsigned current_passes = 0, stale_rejections = 0;
  const auto own = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  cbf::Configuration config;
  config.inter_arm.enabled = true;
  cbf::BodySphere body; body.frame = 8; body.radius = .03;
  config.body_spheres.push_back(body);
  auto peer_config = config;
  peer_config.world_from_base.translation().y() = 1;
  const cbf::ModelView old_view{own.q,own.dq,own.mass,own.coriolis,
                               own.frame_pose,own.frame_jacobian,1000000};
  const auto old_peer = ft::stationary_collision_seed(old_view, peer_config);
  cbf::ConstraintBuilder builder(config);
  cbf::TorqueBounds bounds{-cbf::Vec7::Constant(.35),cbf::Vec7::Constant(.35)};
  robot.setStartupStateObserver([&](const franka::RobotState& received) {
    positions.push_back(received.q[0]);
    auto q = own.q; q[0] = received.q[0];
    const auto observed = ft::panda_model_fixture(q);
    const uint64_t stamp = received.time.toMSec() * 1000000;
    const cbf::ModelView peer_view{observed.q,observed.dq,observed.mass,observed.coriolis,
                                   observed.frame_pose,observed.frame_jacobian,stamp};
    const auto peer = ft::stationary_collision_seed(peer_view, peer_config);
    const cbf::ModelView now{own.q,own.dq,own.mass,own.coriolis,
                            own.frame_pose,own.frame_jacobian,stamp};
    current_passes += builder.build(now,.001,stamp,&bounds,&peer).status == cbf::Status::solved_hard;
    stale_rejections += builder.build(now,.001,stamp,&bounds,&old_peer).status == cbf::Status::stale_peer;
    last_observed.store(received.time.toMSec(),std::memory_order_release);
  });
  ri::Move::Deviation deviation{0,0,0};
  robot.startMotion(ri::Move::ControllerMode::kExternalController,
                    ri::Move::MotionGeneratorMode::kJointVelocity, deviation, deviation);
  ASSERT_EQ(sent.load(), 8u);
  EXPECT_GE(positions.size(), 4u);
  EXPECT_EQ(current_passes, positions.size());
  EXPECT_EQ(stale_rejections, positions.size());
  ASSERT_FALSE(positions.empty());
  EXPECT_LT(positions.front(), positions.back());
}

namespace {
void send_state(RobotMockServer::Socket& udp, uint64_t id, bool moving) {
  ri::RobotState state{};
  state.message_id = id;
  state.motion_generator_mode = moving ? ri::MotionGeneratorMode::kJointVelocity : ri::MotionGeneratorMode::kIdle;
  state.controller_mode = moving ? ri::ControllerMode::kExternalController : ri::ControllerMode::kOther;
  state.robot_mode = moving ? ri::RobotMode::kMove : ri::RobotMode::kIdle;
  udp.sendBytes(&state,sizeof(state));
}
void reply_move(RobotMockServer& server, RobotMockServer::Socket& tcp,
                uint32_t id, ri::Move::Status status) {
  server.sendResponse<ri::Move>(tcp,
      ri::Move::Header(ri::Move::kCommand,id,sizeof(ri::Move::Message<ri::Move::Response>)),
      ri::Move::Response(status));
}
}  // namespace

TEST(InterArmStartupTransport, PublicObserverKeepsExclusiveOwnerAndFinishesNormally) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1",franka::RealtimeConfig::kIgnore);
  unsigned observations=0, commands=0;
  bool controlling=false;
  std::atomic<bool> callback_seen{false};
  server.generic([&](RobotMockServer::Socket& tcp,RobotMockServer::Socket& udp) {
    ri::Move::Header request;
    server.receiveRequest<ri::Move>(tcp,&request);
    reply_move(server,tcp,request.command_id,ri::Move::Status::kMotionStarted);
    uint64_t state_id=100;
    while (!callback_seen.load(std::memory_order_acquire)) {
      send_state(udp,state_id++,true);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ri::RobotCommand output{};
    udp.receiveBytes(&output,sizeof(output));
    EXPECT_TRUE(output.motion.motion_generation_finished);
    for(double torque:output.control.tau_J_d) EXPECT_EQ(torque,0.);
    send_state(udp,state_id,false);
    reply_move(server,tcp,request.command_id,ri::Move::Status::kSuccess);
  }).spinOnce();
  robot.controlWithStartupObserver(
      [&](const franka::RobotState&,franka::Duration) -> franka::Torques {
        controlling=true; ++commands;
        callback_seen.store(true,std::memory_order_release);
        return franka::MotionFinished(franka::Torques({0,0,0,0,0,0,0}));
      },
      [&](const franka::RobotState&) {
        EXPECT_FALSE(controlling);
        ++observations;
        EXPECT_THROW(robot.readOnce(),franka::InvalidOperationException);
      },true,franka::kMaxCutoffFrequency);
  EXPECT_GE(observations,1u);
  EXPECT_EQ(commands,1u);
}

TEST(InterArmStartupTransport, ObserverFailureCancelsBeforeAnyTorqueAndUnlocksRead) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1",franka::RealtimeConfig::kIgnore);
  unsigned commands=0,observations=0;
  server.generic([&](RobotMockServer::Socket& tcp,RobotMockServer::Socket& udp) {
    ri::Move::Header request;
    server.receiveRequest<ri::Move>(tcp,&request);
    send_state(udp,100,false);
    server.handleCommand<ri::StopMove>(tcp,[](const ri::StopMove::Request&) {
      return ri::StopMove::Response(ri::StopMove::Status::kSuccess);
    });
    send_state(udp,101,false);
    reply_move(server,tcp,request.command_id,ri::Move::Status::kSuccess);
  }).spinOnce();
  EXPECT_THROW(robot.controlWithStartupObserver(
      [&](const franka::RobotState&,franka::Duration) -> franka::Torques {
        ++commands; return franka::Torques({0,0,0,0,0,0,0});
      },
      [&](const franka::RobotState&) { ++observations; throw std::runtime_error("injected observer fault"); },
      true,franka::kMaxCutoffFrequency),std::runtime_error);
  EXPECT_EQ(commands,0u);
  EXPECT_EQ(observations,1u);
  server.onSendUDP<ri::RobotState>([](ri::RobotState& s) { s.message_id=102; }).spinOnce();
  EXPECT_NO_THROW(robot.read([](const franka::RobotState&) { return false; }));
  EXPECT_EQ(observations,1u);
}

TEST(InterArmStartupTransport, RejectedMoveAcknowledgementNeverStartsTorqueLoop) {
  RobotMockServer server;
  franka::Robot robot("127.0.0.1",franka::RealtimeConfig::kIgnore);
  unsigned commands=0;
  server.generic([&](RobotMockServer::Socket& tcp,RobotMockServer::Socket& udp) {
    ri::Move::Header request;
    server.receiveRequest<ri::Move>(tcp,&request);
    reply_move(server,tcp,request.command_id,ri::Move::Status::kCommandNotPossibleRejected);
    send_state(udp,100,false);
    server.handleCommand<ri::StopMove>(tcp,[](const ri::StopMove::Request&) {
      return ri::StopMove::Response(ri::StopMove::Status::kSuccess);
    });
    send_state(udp,101,false);
  }).spinOnce();
  EXPECT_THROW(robot.controlWithStartupObserver(
      [&](const franka::RobotState&,franka::Duration) -> franka::Torques {
        ++commands; return franka::Torques({0,0,0,0,0,0,0});
      }, [](const franka::RobotState&) {},true,franka::kMaxCutoffFrequency),franka::CommandException);
  EXPECT_EQ(commands,0u);
}
