#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include "safety_filter/async_safety.hpp"
#include "safety_filter/panda_limits.hpp"
#include "safety_filter/panda_model_fixture.hpp"
namespace sf = safety_filter;
namespace {
constexpr uint64_t start = 1000000000;
std::array<sf::cbf::ArmIdentity, 2> identities() {
  std::array<sf::cbf::ArmIdentity, 2> ids;
  ids[0].arm_id = "left";
  ids[0].base_frame = "left/base";
  ids[1].arm_id = "right";
  ids[1].base_frame = "right/base";
  ids[1].world_from_base.translation().x() = 2;
  return ids;
}
sf::ModelRequest request(int cycle = 1) {
  sf::ModelRequest r;
  r.cycle = cycle;
  r.stamp_ns = start + cycle * 1000000;
  r.epoch = 1;
  r.state = sf::panda_model_fixture(Eigen::Map<const sf::cbf::Vector7d>(sf::kHome.data()));
  return r;
}
sf::SafetyReply success(const sf::ModelRequest& r, int arm = 0) {
  sf::SafetyReply reply;
  reply.cycles[arm] = r.cycle;
  reply.source_ns[arm] = r.stamp_ns;
  reply.completed_ns = r.stamp_ns;
  reply.epoch[arm] = r.epoch;
  reply.source_q[arm] = r.state.q;
  reply.source_dq[arm] = r.state.dq;
  reply.nominal.segment<7>(arm * 7) = r.nominal;
  reply.result.safe_torque = reply.nominal;
  reply.result.solver_succeeded = true;
  return reply;
}
struct Fixture {
  sf::CoordinatorParameters limits;
  sf::cbf::SafetyParameters safety;
  sf::AsyncSafetyController local;
  Fixture(bool enabled = true, bool dual = false) {
    limits.asynchronous = true;
    limits.active = {true, dual};
    safety.active_arms = limits.active;
    safety.enabled = enabled;
    safety.self_collision_enabled = safety.inter_arm_collision_enabled = false;
  }
  void configure(int arm = 0, const std::array<sf::cbf::ArmIdentity, 2>& ids = identities()) {
    std::string error;
    ASSERT_TRUE(local.configure(arm, limits, safety, ids, &error)) << error;
  }
};
}  // namespace
TEST(AsyncSafety, PendingModeDoesNotDisarmEveryEngagementAndNeverUsesOldEpoch) {
  Fixture f;
  f.configure();
  auto r = request();
  r.motion_active = true;
  r.nominal[0] = .1;
  auto old = success(r);
  old.epoch[0] = 0;
  old.result.safe_torque[0] = .3;
  auto out = f.local.update(r, nullptr, &old, r.stamp_ns);
  EXPECT_FALSE(out.worker_usable);
  EXPECT_FALSE(out.degraded);
  EXPECT_NEAR(out.reply.result.safe_torque[0], 0, 1e-9);
  r = request(6);
  r.motion_active = true;
  r.nominal[0] = .1;
  auto current = success(r);
  out = f.local.update(r, nullptr, &current, r.stamp_ns);
  EXPECT_TRUE(out.worker_usable);
  EXPECT_FALSE(out.degraded);
  EXPECT_NEAR(out.reply.result.safe_torque[0], .1, 1e-9);
  // Release/mode change cannot reuse a torque from the active context.
  r = request(7);
  r.epoch = 2;
  out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_FALSE(out.worker_usable);
  EXPECT_NEAR(out.reply.result.safe_torque[0], 0, 1e-9);
}
TEST(AsyncSafety, HealthyReleasedDwellCountsTowardFreshReengagement) {
  Fixture f(false);
  f.limits.recovery_dwell_ns = 100000000;
  f.configure();
  auto r = request();
  r.motion_active = true;
  ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
  ++r.cycle;
  r.stamp_ns += 1000000;
  r.state.mass(0, 0) = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
  ASSERT_TRUE(f.local.model_reused());
  auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  ASSERT_TRUE(out.degraded);
  ASSERT_FALSE(out.reply.result.fatal_solver_fault);
  r.state.dq.setZero();
  r.nominal.setZero();
  r.motion_active = false;
  r.epoch = 2;
  for (int cycle = 3; cycle <= 150; ++cycle) {
    r.cycle = cycle;
    r.stamp_ns = start + cycle * 1000000;
    r.release_source_ns = r.stamp_ns;
    ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
    out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
    ASSERT_TRUE(out.worker_usable);
    ASSERT_FALSE(out.reply.result.fatal_solver_fault);
    EXPECT_TRUE(out.degraded);  // Release alone never resumes movement.
  }
  ++r.cycle;
  r.stamp_ns += 1000000;
  r.epoch = 3;
  r.motion_active = true;
  r.nominal[0] = .1;
  out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_FALSE(out.degraded);
  EXPECT_NEAR(out.reply.result.safe_torque[0], .1, 1e-9);
}
TEST(AsyncSafety, CbfRecoveryDwellSurvivesModeHandoffWithoutReusingItsTorque) {
  Fixture f;
  f.configure();
  auto r = request();
  r.motion_active = true;
  auto bad = success(r);
  bad.result.solver_succeeded = false;
  auto out = f.local.update(r, nullptr, &bad, r.stamp_ns);
  ASSERT_TRUE(out.degraded);
  r.motion_active = false;
  r.epoch = 2;
  for (int cycle = 2; cycle <= 150; ++cycle) {
    r.cycle = cycle;
    r.stamp_ns = start + cycle * 1000000;
    r.release_source_ns = r.stamp_ns;
    auto good = success(r);
    out = f.local.update(r, nullptr, &good, r.stamp_ns);
  }
  ++r.cycle;
  r.stamp_ns += 1000000;
  r.epoch = 3;
  r.motion_active = true;
  r.nominal[0] = .2;
  out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_FALSE(out.worker_usable);
  EXPECT_TRUE(out.degraded);  // No torque reuse across the epoch.
  r.cycle += 5;
  r.stamp_ns += 5000000;
  auto good = success(r);
  out = f.local.update(r, nullptr, &good, r.stamp_ns);
  EXPECT_TRUE(out.worker_usable);
  EXPECT_FALSE(out.degraded);  // No second 100 ms dwell after the mode handoff.
  EXPECT_NEAR(out.reply.result.safe_torque[0], .2, 1e-9);
}
TEST(AsyncSafety, HealthBeforeTheFaultDoesNotCountAsRecoveryDwell) {
  Fixture f(false);
  f.limits.recovery_dwell_ns = 100000000;
  f.configure();
  auto r = request();
  for (int cycle = 1; cycle <= 150; ++cycle) {
    r.cycle = cycle;
    r.stamp_ns = start + cycle * 1000000;
    ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
    auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
    ASSERT_FALSE(out.degraded);
  }
  ++r.cycle;
  r.stamp_ns += 1000000;
  r.motion_active = true;
  r.state.mass(0, 0) = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
  auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  ASSERT_TRUE(out.degraded);
  ++r.cycle;
  r.stamp_ns += 1000000;
  r.release_source_ns = r.stamp_ns;
  r.motion_active = false;
  r.epoch = 2;
  r.nominal.setZero();
  r.state.dq.setZero();
  ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
  f.local.update(r, nullptr, nullptr, r.stamp_ns);
  ++r.cycle;
  r.stamp_ns += 1000000;
  r.epoch = 3;
  r.motion_active = true;
  out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_TRUE(out.degraded);  // Only 1 ms of new health, despite the old 150 ms.
}
TEST(AsyncSafety, FeasibleCurrentCorrectionDoesNotLatchBrakingDuringChangingTargets) {
  Fixture f;
  f.configure();
  auto r = request();
  r.motion_active = true;
  r.state.previous_torque[0] = 6.8;
  r.nominal[0] = 6.8;
  auto worker = success(r);
  auto out = f.local.update(r, nullptr, &worker, r.stamp_ns);
  ASSERT_FALSE(out.degraded);
  ++r.cycle;
  r.stamp_ns += 1000000;
  // The desired torque rises before the next worker solve. Its rate-bounded
  // candidate is 7.15 Nm, slightly above the 7 rad/s^2 acceleration row for
  // this identity-mass fixture. A feasible current correction exists at 7 Nm.
  r.nominal[0] = 20;
  out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_FALSE(out.degraded);
  EXPECT_FALSE(out.reply.result.fallback_active);
  EXPECT_TRUE(out.reply.result.solver_succeeded);
  EXPECT_LE(out.reply.result.maximum_constraint_violation, 1e-3);
  EXPECT_GE(out.reply.result.safe_torque[0], 6.45);
  EXPECT_LE(out.reply.result.safe_torque[0], 7.001);
}

TEST(AsyncSafety, RealWorkerWithCoupledLightWristTracksChangingTargetsWithoutLatchingHold) {
  Fixture f;
  f.safety.singularity_enabled = true;
  f.safety.self_collision_enabled = true;
  f.configure();
  sf::cbf::CbfSafetyFilter worker;
  std::string error;
  ASSERT_TRUE(worker.configure(f.safety, identities(), &error)) << error;
  auto r = request();
  r.state.mass.diagonal() << 2.5, 2, 1.2, .7, .03, .02, .006;
  sf::cbf::Matrix7d l = sf::cbf::Matrix7d::Identity();
  l(3, 1) = .15;
  l(5, 3) = .2;
  l(6, 2) = .1;
  r.state.mass = l * r.state.mass * l.transpose();
  r.motion_active = true;
  sf::SafetyReply reply;
  double maximum_command = 0;
  for (int cycle = 1; cycle <= 1000; ++cycle) {
    r.cycle = cycle;
    r.stamp_ns = start + cycle * 1000000;
    for (int joint = 0; joint < 7; ++joint)
      r.nominal[joint] = 2 * std::sin(.01 * cycle + joint);
    const sf::SafetyReply* incoming = nullptr;
    if (cycle % 5 == 1) {
      sf::cbf::DualArmState model;
      model.arm[0] = r.state;
      sf::cbf::Vector14d nominal = sf::cbf::Vector14d::Zero();
      nominal.head<7>() = r.nominal;
      reply = success(r);
      reply.result = worker.filter(nominal, model, .005, {true, false});
      ASSERT_TRUE(reply.result.solver_succeeded) << reply.result.solver_status;
      incoming = &reply;
    }
    const auto out = f.local.update(r, nullptr, incoming, r.stamp_ns);
    ASSERT_TRUE(out.worker_usable);
    ASSERT_FALSE(out.degraded) << "cycle=" << cycle << " status=" << out.reply.result.solver_status;
    EXPECT_FALSE(out.reply.result.fallback_active);
    EXPECT_LE(out.reply.result.maximum_constraint_violation, 1e-3);
    maximum_command = std::max(maximum_command, out.reply.result.safe_torque.norm());
    r.state.previous_torque = out.reply.result.safe_torque.head<7>();
  }
  EXPECT_GT(maximum_command, .1);
}
TEST(AsyncSafety, FailedWorkerBrakesAndHoldsUntilReleaseReengagementAndHealthyDwell) {
  Fixture f;
  f.limits.recovery_dwell_ns = 2000000;
  f.configure();
  auto r = request();
  r.motion_active = true;
  r.state.dq[0] = .1;
  auto failed = success(r);
  failed.result.solver_succeeded = false;
  failed.result.fallback_active = true;
  auto out = f.local.update(r, nullptr, &failed, r.stamp_ns);
  EXPECT_TRUE(out.degraded);
  EXPECT_TRUE(out.entered_degraded);
  EXPECT_STREQ(out.degraded_cause, "solver_not_verified");
  EXPECT_EQ(out.degraded_since_cycle, r.cycle);
  EXPECT_LT(out.reply.result.safe_torque[0], 0);
  EXPECT_FALSE(out.reply.result.fatal_solver_fault);
  // Healthy results and held trigger alone cannot resume tracking.
  for (int i = 2; i < 8; ++i) {
    r = request(i);
    r.motion_active = true;
    auto ok = success(r);
    out = f.local.update(r, nullptr, &ok, r.stamp_ns);
    EXPECT_TRUE(out.degraded);
    EXPECT_STREQ(out.degraded_cause, "solver_not_verified");
    EXPECT_EQ(out.degraded_since_cycle, 1u);
  }
  r = request(8);
  r.release_source_ns = r.stamp_ns;
  r.epoch = 2;
  auto ok = success(r);
  out = f.local.update(r, nullptr, &ok, r.stamp_ns);
  EXPECT_TRUE(out.degraded);
  for (int i = 9; i < 14; ++i) {
    r = request(i);
    r.motion_active = true;
    r.release_source_ns = start + 8000000;
    r.epoch = 3;
    ok = success(r);
    out = f.local.update(r, nullptr, &ok, r.stamp_ns);
  }
  EXPECT_FALSE(out.degraded);
  EXPECT_TRUE(out.worker_usable);
}
TEST(AsyncSafety, ExpiredAndOutOfOrderResultsDoNotRefreshTheirSourceAge) {
  Fixture f;
  f.configure();
  auto r = request();
  r.motion_active = true;
  auto old = success(r);
  f.local.update(r, nullptr, &old, r.stamp_ns);
  r = request(60);
  r.motion_active = true;
  old.completed_ns = r.stamp_ns;
  auto out = f.local.update(r, nullptr, &old, r.stamp_ns);
  EXPECT_FALSE(out.worker_usable);
  EXPECT_TRUE(out.degraded);
  EXPECT_EQ(out.worker_age_ns, 59000000u);
  auto newer = success(r);
  f.local.update(r, nullptr, &newer, r.stamp_ns);
  r = request(61);
  r.motion_active = true;
  out = f.local.update(r, nullptr, &old, r.stamp_ns);
  EXPECT_TRUE(out.worker_usable);
  EXPECT_EQ(out.worker_age_ns, 1000000u);
}
TEST(AsyncSafety, WorldTableRemainsMandatoryWithCbfDisabledAndProtectsLinksNotJustTool) {
  Fixture f(false);
  f.safety.table_height = .1;
  f.configure();
  auto r = request();
  r.motion_active = true;
  r.nominal[0] = -.3;
  r.state.dq[0] = -.02;
  for (int frame = 0; frame < 9; ++frame) {
    r.state.frame_pose[frame].setIdentity();
    r.state.frame_pose[frame].translation().z() = .5;
    r.state.frame_jacobian[frame].setZero();
    r.state.frame_jacobian[frame](2, 0) = 1;
  }
  auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_GT(out.reply.result.table_constraints, 0);
  EXPECT_GT(out.reply.result.safe_torque[0], 0);
  EXPECT_FALSE(out.reply.result.fatal_solver_fault);
  EXPECT_FALSE(out.degraded);  // A verified correction is ordinary intervention.
  EXPECT_TRUE(out.reply.result.solver_succeeded);
  EXPECT_LE(out.reply.result.maximum_constraint_violation, 1e-3);
  auto ids = identities();
  ids[0].world_from_base.translation().z() = 1;
  Fixture high(false);
  high.safety.table_height = .1;
  high.configure(0, ids);
  out = high.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_GT(out.reply.result.minimum_table_clearance, 1.0);
  EXPECT_NEAR(out.reply.result.safe_torque[0], -.3, 1e-9);
}
TEST(AsyncSafety, SecondaryInfeasibilityHoldsWithoutTerminatingOwnership) {
  Fixture f;
  f.safety.obstacle_collision_enabled = true;
  f.safety.obstacle_centers = {.4, 0, .4};
  f.safety.obstacle_radii = {3};
  f.configure();
  auto r = request();
  r.motion_active = true;
  auto failed = success(r);
  failed.result.solver_succeeded = false;
  auto out = f.local.update(r, nullptr, &failed, r.stamp_ns);
  EXPECT_TRUE(out.degraded);
  EXPECT_TRUE(out.reply.result.fallback_active);
  EXPECT_FALSE(out.reply.result.fatal_input_fault);
  EXPECT_FALSE(out.reply.result.fatal_solver_fault);
  EXPECT_TRUE(out.reply.result.safe_torque.allFinite());
  EXPECT_LE(out.reply.result.safe_torque.cwiseAbs().maxCoeff(), .35 + 1e-9);
}
TEST(AsyncSafety, MandatoryInfeasibilityIsAnExceptionalFault) {
  Fixture f(false);
  f.safety.table_height = 2;
  f.configure();
  auto r = request();
  auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
  EXPECT_TRUE(out.reply.result.fatal_solver_fault);
}
TEST(AsyncSafety, HorizontalCapsuleEndpointCrossingDoesNotCreateFalseTableAcceleration) {
  Fixture f(false);
  f.configure();
  auto r = request();
  r.state.mass.setIdentity();
  r.state.coriolis.setZero();
  r.state.dq.setZero();
  r.state.dq[0] = 1;
  for (int frame = 0; frame < 9; ++frame) {
    r.state.frame_pose[frame].setIdentity();
    r.state.frame_pose[frame].translation().z() = .6;
    r.state.frame_jacobian[frame].setZero();
  }
  r.state.frame_pose[7].translation().z() = .3;
  r.state.frame_jacobian[7](3, 0) = 1;
  // The Hand's horizontal capsules change which endpoint is lowest. Each
  // endpoint has smooth motion; differencing their two different Jacobians
  // as if they were one point produces an artificial 1/dt acceleration.
  for (double angle : {-.0005, .0005}) {
    r.state.frame_pose[7].linear() =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
    EXPECT_GT(out.reply.result.minimum_table_clearance, .15);
    EXPECT_FALSE(out.reply.result.fatal_solver_fault) << out.reply.result.solver_status;
    r.stamp_ns += 1000000;
    ++r.cycle;
    r.state.q[0] += .001;
  }
}
TEST(AsyncSafety, IsolatedModelGlitchHasBoundedGraceWithoutRefreshingTimestamp) {
  Fixture f;
  f.configure();
  auto r = request();
  ASSERT_TRUE(f.local.prepare_model(r.state, r.stamp_ns));
  const auto good = r.state;
  r.state.mass(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(f.local.prepare_model(r.state, r.stamp_ns + 1000000));
  EXPECT_TRUE(f.local.model_reused());
  r.state = good;
  r.state.mass(0, 0) = -1;
  EXPECT_FALSE(f.local.prepare_model(r.state, r.stamp_ns + 4000000));
  r.state = good;
  r.state.q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(f.local.prepare_model(r.state, r.stamp_ns + 1000000));
}
TEST(AsyncSafety, DualPeerFreshnessDoesNotBlockEitherOwner) {
  Fixture f(true, true);
  f.safety.inter_arm_collision_enabled = true;
  f.configure();
  auto r = request();
  r.motion_active = true;
  auto ok = success(r);
  auto out = f.local.update(r, nullptr, &ok, r.stamp_ns);
  EXPECT_FALSE(out.peer_usable);
  EXPECT_FALSE(out.reply.result.fatal_solver_fault);
  auto peer = request();
  out = f.local.update(r, &peer, nullptr, r.stamp_ns);
  EXPECT_TRUE(out.peer_usable);
  r = request(60);
  r.motion_active = true;
  ok = success(r);
  out = f.local.update(r, &peer, &ok, r.stamp_ns);
  EXPECT_FALSE(out.peer_usable);
  EXPECT_TRUE(out.degraded);
}
TEST(AsyncSafety, CallbackProgressDoesNotWaitForWorkerThread) {
  Fixture f;
  f.configure();
  std::promise<void> release, started;
  auto blocker = release.get_future();
  auto ready = started.get_future();
  std::thread worker([&] {
    started.set_value();
    blocker.wait();
  });
  ready.wait();
  int progressed = 0;
  for (int i = 1; i <= 80; ++i) {
    auto r = request(i);
    auto out = f.local.update(r, nullptr, nullptr, r.stamp_ns);
    EXPECT_FALSE(out.reply.result.fatal_solver_fault);
    ++progressed;
  }
  release.set_value();
  worker.join();
  EXPECT_EQ(progressed, 80);
}
TEST(AsyncCoordinator, SlowAndFailedWorkerNeverLatchDeadlineOrSolverStops) {
  sf::SafetyCoordinator coordinator;
  sf::CoordinatorParameters p;
  p.asynchronous = true;
  p.filter_deadline_ns = 1;
  p.result_deadline_ns = 2;
  p.maximum_failures = 1;
  sf::cbf::SafetyParameters safety;
  safety.active_arms = p.active;
  safety.obstacle_collision_enabled = true;
  safety.obstacle_centers = {.4, 0, .4};
  safety.obstacle_radii = {3};
  std::string error;
  ASSERT_TRUE(coordinator.configure(p, safety, identities(), &error)) << error;
  auto r = request();
  coordinator.submit(0, r);
  EXPECT_TRUE(coordinator.pump(r.stamp_ns));
  EXPECT_EQ(coordinator.stopped(), sf::SafetyStop::none);
  sf::SafetyReply reply;
  ASSERT_TRUE(coordinator.receive(0, reply));
  EXPECT_TRUE(reply.result.fallback_active);
  for (int i = 2; i < 20; ++i) {
    r = request(i);
    coordinator.submit(0, r);
    coordinator.pump(r.stamp_ns);
    EXPECT_EQ(coordinator.stopped(), sf::SafetyStop::none);
  }
}
TEST(AsyncCoordinator, DualRequestsAreLatestOnlyAndFinishNeedsBothOwners) {
  sf::SafetyCoordinator c;
  sf::CoordinatorParameters p;
  p.asynchronous = true;
  p.active = {true, true};
  sf::cbf::SafetyParameters safety;
  safety.active_arms = p.active;
  std::string error;
  ASSERT_TRUE(c.configure(p, safety, identities(), &error)) << error;
  auto left = request();
  left.finish_requested = true;
  c.submit(0, left);
  EXPECT_FALSE(c.may_finish());
  EXPECT_FALSE(c.pump(left.stamp_ns));
  auto right = request(3);
  right.finish_requested = true;
  c.submit(1, right);
  left = request(3);
  left.finish_requested = true;
  c.submit(0, left);
  EXPECT_TRUE(c.may_finish());
  EXPECT_TRUE(c.pump(left.stamp_ns));
  sf::SafetyReply reply;
  ASSERT_TRUE(c.receive(0, reply));
  EXPECT_EQ(reply.cycles[0], 3u);
  EXPECT_EQ(reply.cycles[1], 3u);
  sf::ModelRequest peer;
  ASSERT_TRUE(c.peer(0, peer));
  EXPECT_EQ(peer.cycle, 3u);
}
