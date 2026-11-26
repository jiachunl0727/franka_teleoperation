#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <limits>
#include <thread>

#include "teleop_controllers/cartesian_pd_controller.hpp"
#include "teleop_controllers/controller_manager.hpp"
#include "teleop_controllers/damping_controller.hpp"
#include "teleop_controllers/handoff.hpp"
#include "teleop_controllers/orientation_only_controller.hpp"
#include "teleop_controllers/position_hold_controller.hpp"
namespace ft = teleop_controllers;
namespace local = teleop_controllers;
namespace {
constexpr uint64_t now = 1000000000;
ft::Command command(uint64_t seq, bool enabled, bool released = false) {
  ft::Command c;
  std::strcpy(c.session.data(), "first");
  c.sequence = seq;
  c.source_ns = now + seq;
  c.receipt_ns = now + seq;
  c.valid = true;
  c.enabled = enabled;
  c.released = released;
  c.position << 0.4, 0, 0.4;
  return c;
}
ft::ArmState state() {
  ft::ArmState s;
  s.position << 0.4, 0, 0.4;
  s.jacobian.leftCols<6>().setIdentity();
  return s;
}
struct Fixture {
  ft::CartesianPdParameters pd;
  ft::CartesianPdController cartesian{pd};
  ft::OrientationOnlyController orientation_only{pd};
  ft::DampingController damping{pd.gain_translation, pd.gain_rotation};
  ft::PositionHoldController position_hold{ft::PositionHoldParameters{}};
  ft::ControllerManager manager;
  explicit Fixture(const ft::ManagerParameters& p = {})
      : manager(p, ft::HomingParameters{}, cartesian, orientation_only, position_hold, damping) {}
  void accept(const ft::Command& c, uint64_t stamp) { manager.accept(c, stamp); }
  ft::Output update(const ft::ArmState& s, uint64_t stamp, double period) {
    return manager.update(s, stamp, period).control;
  }
};
void arm(Fixture& core) {
  core.accept(command(1, false, true), now + 1);
  core.accept(command(2, true), now + 2);
}
}  // namespace
TEST(Manager, OrientationModeCapturesPositionAndSwitchesWithoutTriggerRelease) {
  Fixture core;
  arm(core);
  auto measured = state();
  auto full = command(3, true);
  full.position.x() = .45;
  core.accept(full, now + 3);
  EXPECT_DOUBLE_EQ(core.update(measured, now + 3, .001).target_position.x(), .45);

  auto orientation = command(4, true);
  orientation.orientation_only = true;
  orientation.position.x() = .5;
  orientation.twist.x() = .2;
  core.accept(orientation, now + 4);
  auto output = core.update(measured, now + 4, .001);
  EXPECT_TRUE(output.motion_enabled);
  EXPECT_EQ(output.reason, ft::Reason::orientation_only);
  EXPECT_DOUBLE_EQ(output.target_position.x(), .4);
  EXPECT_DOUBLE_EQ(output.desired_twist.x(), 0.0);
  measured.position.x() = .41;
  core.accept(command(5, true), now + 5);
  output = core.update(measured, now + 5, .001);
  EXPECT_TRUE(output.motion_enabled);
  EXPECT_DOUBLE_EQ(output.target_position.x(), .4);
}
TEST(Manager, ReleaseIsRequiredAndNominalLawIsPreserved) {
  Fixture core(ft::ManagerParameters{});
  auto s = state();
  core.accept(command(1, true), now + 1);
  EXPECT_FALSE(core.update(s, now + 1, .001).motion_enabled);
  core.accept(command(2, false, true), now + 2);
  auto c = command(3, true);
  c.position.x() += .01;
  c.twist.x() = .1;
  core.accept(c, now + 3);
  s.dq[0] = .02;
  s.coriolis[0] = .5;
  const auto out = core.update(s, now + 3, .001);
  ASSERT_TRUE(out.motion_enabled);
  EXPECT_DOUBLE_EQ(out.desired_twist.x(), .1);
  EXPECT_NEAR(out.target_position.x(), .41, 1e-12);
  // With optional limits off, the original pd law sees the mapped input.
  EXPECT_NEAR(out.nominal[0], 12.1, 1e-10);
  // Only the runtime fills the final actuator-clipped torque.
  EXPECT_TRUE(out.torque.isZero());
}
TEST(Manager, SmallPoseWithImplausibleVelocityCannotDriveTheArm) {
  Fixture core;
  arm(core);
  auto c = command(3, true);
  c.position.x() += .005;
  c.twist.tail<3>().setConstant(100.);
  core.accept(c, now + 3);
  const auto out = core.update(state(), now + 3, .001);
  EXPECT_FALSE(out.motion_enabled);
  EXPECT_TRUE(out.desired_twist.isZero());
}
TEST(Manager, StaleTargetIsRemovedAndPacketsCannotResume) {
  Fixture core(ft::ManagerParameters{});
  arm(core);
  auto s = state();
  s.previous_torque[0] = 5;
  const auto out = core.update(s, now + 100000003, .001);
  EXPECT_FALSE(out.motion_enabled);
  EXPECT_EQ(out.reason, ft::Reason::position_hold);
  EXPECT_EQ(out.target_position, s.position);
  EXPECT_EQ(out.desired_twist, ft::Vector6::Zero());
  EXPECT_TRUE(out.nominal.isZero());
  auto c = command(3, true);
  c.source_ns = c.receipt_ns = now + 100000004;
  core.accept(c, c.receipt_ns);
  EXPECT_FALSE(core.update(s, c.receipt_ns, .001).motion_enabled);
  c.sequence++;
  c.source_ns++;
  c.receipt_ns++;
  c.enabled = false;
  c.released = true;
  core.accept(c, c.receipt_ns);
  c.sequence++;
  c.source_ns++;
  c.receipt_ns++;
  c.enabled = true;
  c.released = false;
  core.accept(c, c.receipt_ns);
  EXPECT_TRUE(core.update(s, c.receipt_ns, .001).motion_enabled);
}
TEST(Manager, FaultDisarmIsNeverARelease) {
  Fixture core(ft::ManagerParameters{});
  arm(core);
  auto c = command(3, false, true);
  c.valid = false;
  core.accept(c, now + 3);
  core.accept(command(4, true), now + 4);
  EXPECT_FALSE(core.update(state(), now + 4, .001).motion_enabled);
}
TEST(Manager, SessionReferenceOrderingAndClockFaultsRequireRelease) {
  for (int scenario = 0; scenario < 7; ++scenario) {
    Fixture core(ft::ManagerParameters{});
    arm(core);
    auto c = command(3, true);
    if (scenario == 0)
      std::strcpy(c.session.data(), "second");
    if (scenario == 1)
      c.generation++;
    if (scenario == 2)
      c.sequence = 2;
    if (scenario == 3)
      c.source_ns = now + 2;
    if (scenario == 4)
      c.uncertainty_ns = 10000001;
    if (scenario == 5)
      c.source_ns = now - 100000001;
    if (scenario == 6)
      c.source_ns = now + 20000000;
    core.accept(c, now + 3);
    EXPECT_FALSE(core.update(state(), now + 3, .001).motion_enabled) << scenario;
  }
}
TEST(Manager, ReleaseRepeatsDoNotRefreshSourceAge) {
  Fixture core(ft::ManagerParameters{});
  auto c = command(1, false, true);
  core.accept(c, now + 1);
  c.receipt_ns = now + 90000000;
  core.accept(c, c.receipt_ns);
  auto enabled = command(2, true);
  enabled.source_ns = enabled.receipt_ns = now + 100000002;
  core.update(state(), enabled.receipt_ns, .001);
  core.accept(enabled, enabled.receipt_ns);
  EXPECT_FALSE(core.update(state(), enabled.receipt_ns, .001).motion_enabled);
}
TEST(Manager, RemovedOuterEnvelopeAllowsFiniteTargetsAndRejectsNonFiniteTargets) {
  Fixture finite_target;
  arm(finite_target);
  auto command_far_away = command(3, true);
  command_far_away.position.x() = 10;
  finite_target.accept(command_far_away, now + 3);
  const auto allowed = finite_target.update(state(), now + 3, .001);
  EXPECT_TRUE(allowed.motion_enabled);
  EXPECT_DOUBLE_EQ(allowed.target_position.x(), 10);

  Fixture non_finite_target;
  arm(non_finite_target);
  auto invalid = command(3, true);
  invalid.position.x() = std::numeric_limits<double>::quiet_NaN();
  non_finite_target.accept(invalid, now + 3);
  const auto rejected = non_finite_target.update(state(), now + 3, .001);
  EXPECT_FALSE(rejected.motion_enabled);
  EXPECT_EQ(rejected.desired_twist, ft::Vector6::Zero());
}
TEST(Manager, CoriolisConventionIsPreservedButInvalidActuatorIntersectionIsFatal) {
  Fixture core(ft::ManagerParameters{});
  arm(core);
  auto s = state();
  s.q = core.pd.home_joint_position;
  s.coriolis.setConstant(10000);
  s.previous_torque = ft::ManagerParameters{}.torque_limits - ft::Vector7::Constant(.2);
  auto out = core.update(s, now + 2, .001);
  EXPECT_TRUE(out.nominal.isConstant(10000));
  EXPECT_TRUE(out.torque.isZero());
  s.previous_torque[0] = 100;
  EXPECT_EQ(core.update(s, now + 2, .001).reason, ft::Reason::invalid_state);
}
TEST(Manager, QuaternionSignDoesNotChangeTorque) {
  Fixture first(ft::ManagerParameters{}), second(ft::ManagerParameters{});
  arm(first);
  arm(second);
  auto c = command(3, true);
  c.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitZ()));
  first.accept(c, now + 3);
  c.orientation.coeffs() *= -1;
  second.accept(c, now + 3);
  EXPECT_TRUE(first.update(state(), now + 3, .001)
                  .nominal.isApprox(second.update(state(), now + 3, .001).nominal));
}
TEST(Manager, ParametersRejectUnapprovedCeilingChangesAndNaNs) {
  ft::ManagerParameters p;
  ASSERT_TRUE(p.valid());
  p.torque_rate = 1001;
  EXPECT_FALSE(p.valid());
  p = ft::ManagerParameters{};
  p.torque_limits[6] = 13;
  EXPECT_FALSE(p.valid());
  ft::CartesianPdParameters gains;
  ASSERT_TRUE(gains.valid());
  gains.gain_translation = 0;
  EXPECT_FALSE(gains.valid());
  gains = {};
  gains.lambda_rotation = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gains.valid());
}
TEST(Manager, OverflowSizedFiniteVelocityIsRejectedBeforeComputingNominal) {
  Fixture core(ft::ManagerParameters{});
  arm(core);
  auto c = command(3, true);
  c.twist[0] = 1e308;
  core.accept(c, now + 3);
  const auto out = core.update(state(), now + 3, .001);
  EXPECT_EQ(out.reason, ft::Reason::input_velocity_limit);
  EXPECT_FALSE(out.motion_enabled);
}
TEST(Handoff, QueueIsBoundedAndPreservesOrder) {
  local::SpscQueue<int, 4> queue;
  ASSERT_TRUE(queue.push(1));
  ASSERT_TRUE(queue.push(2));
  ASSERT_TRUE(queue.push(3));
  EXPECT_FALSE(queue.push(4));
  int out;
  for (int expected = 1; expected < 4; ++expected) {
    ASSERT_TRUE(queue.pop(out));
    EXPECT_EQ(out, expected);
  }
  EXPECT_FALSE(queue.pop(out));
}
TEST(Handoff, ConcurrentSamplesRemainAtomic) {
  using Sample = std::array<uint64_t, 16>;
  local::LatestSnapshot<Sample> snapshot;
  std::atomic<bool> done{false};
  std::thread producer([&] {
    for (uint64_t n = 1; n <= 100000; ++n) {
      Sample s;
      s.fill(n);
      snapshot.publish(s);
    }
    done.store(true, std::memory_order_release);
  });
  uint64_t previous = 0;
  while (!done.load(std::memory_order_acquire)) {
    Sample s;
    if (snapshot.read(s)) {
      EXPECT_GT(s[0], previous);
      previous = s[0];
      for (auto value : s)
        EXPECT_EQ(value, s[0]);
    }
  }
  producer.join();
  Sample s;
  if (snapshot.read(s)) {
    for (auto value : s)
      EXPECT_EQ(value, uint64_t{100000});
  }
}

TEST(Manager, ReportsSpecificFatalCauseWithoutRelaxingChecks) {
  for (int scenario = 0; scenario < 4; ++scenario) {
    Fixture core(ft::ManagerParameters{});
    auto s = state();
    double period = .001;
    ft::StateFault expected;
    if (scenario == 0) {
      s.q[0] = std::numeric_limits<double>::quiet_NaN();
      expected = ft::StateFault::nonfinite_state;
    }
    if (scenario == 1) {
      s.orientation.w() = 2;
      expected = ft::StateFault::orientation;
    }
    if (scenario == 2) {
      period = .021;
      expected = ft::StateFault::period;
    }
    if (scenario == 3) {
      s.previous_torque[0] = 90;
      expected = ft::StateFault::actuator_interval;
    }
    const auto out = core.update(s, 1000000000, period);
    EXPECT_EQ(out.reason, ft::Reason::invalid_state);
    EXPECT_EQ(out.state_fault, expected);
    EXPECT_FALSE(out.motion_enabled);
  }
}

namespace {
struct ProbeController : ft::TorqueController {
  int calls{0}, resets{0};
  ft::NominalTorque result{ft::Vector7::Constant(3), ft::StateFault::none};
  ft::CartesianReference last_reference;
  void reset(const ft::ArmState&) noexcept override { ++resets; }
  ft::NominalTorque compute(const ft::ArmState&, const ft::ControlReference& reference,
                            double) noexcept override {
    ++calls;
    if (const auto* cartesian = std::get_if<ft::CartesianReference>(&reference))
      last_reference = *cartesian;
    return result;
  }
};
}  // namespace

TEST(Manager, InjectedLawRunsOnlyWhenArmedAndResetsOnActivation) {
  ProbeController law;
  ft::DampingController damping(20, 2.5);
  ft::PositionHoldController hold({});
  ft::ControllerManager manager({}, {}, law, hold, damping);
  auto s = state();
  s.dq[0] = .1;
  s.coriolis[0] = .5;
  EXPECT_NEAR(manager.update(s, now, .001).control.nominal[0], -4.5, 1e-10);
  EXPECT_EQ(law.calls, 0);
  manager.accept(command(1, false, true), now + 1);
  manager.accept(command(2, true), now + 2);
  EXPECT_TRUE(manager.update(s, now + 2, .001).control.nominal.isConstant(3));
  EXPECT_EQ(law.calls, 1);
  EXPECT_EQ(law.resets, 1);
  manager.update(s, now + 3, .001);
  EXPECT_EQ(law.resets, 1);
  manager.fault(ft::Reason::invalid_command);
  EXPECT_FALSE(manager.update(s, now + 4, .001).control.motion_enabled);
  EXPECT_EQ(law.calls, 2);
  manager.accept(command(3, true), now + 5);
  EXPECT_FALSE(manager.update(s, now + 5, .001).control.motion_enabled);
  manager.accept(command(4, false, true), now + 6);
  manager.accept(command(5, true), now + 7);
  manager.update(s, now + 7, .001);
  EXPECT_EQ(law.resets, 2);
  manager.shutdown();
  manager.accept(command(6, false, true), now + 8);
  manager.accept(command(7, true), now + 9);
  EXPECT_EQ(manager.update(s, now + 9, .001).control.reason, ft::Reason::shutdown);
  EXPECT_EQ(law.calls, 3);
}

TEST(Manager, HomingOwnsNominalUntilControlledStopAndRequiresFreshReengagement) {
  ProbeController law;
  ft::DampingController damping(20, 2.5);
  ft::PositionHoldController hold({});
  ft::ControllerManager manager({}, {}, law, hold, damping);
  auto s = state();
  s.q = Eigen::Map<const ft::Vector7>(teleop_controllers::kHome.data());
  s.q[0] += .2;
  uint64_t stamp = now, sequence = 0;
  auto home_input = [&](bool button, bool trigger_released = true) {
    ft::HomingInput input;
    std::strcpy(input.session.data(), "home");
    input.source_ns = input.receipt_ns = stamp;
    input.sequence = ++sequence;
    input.valid = true;
    input.button_pressed = button;
    input.trigger_released = trigger_released;
    manager.homing_input(input, stamp);
  };
  home_input(false);
  for (int n = 0; n <= 100; ++n) {
    stamp += 10000000;
    home_input(true);
    manager.update(s, stamp, .01);
  }
  stamp += 10000000;
  home_input(false);
  auto result = manager.update(s, stamp, .001);
  ASSERT_TRUE(result.homing.active);
  EXPECT_EQ(result.control.reason, ft::Reason::homing);
  EXPECT_EQ(law.calls, 0);
  // A valid enabled command cannot displace an active home/controlled stop.
  auto c = command(1, false, true);
  c.source_ns = c.receipt_ns = ++stamp;
  manager.accept(c, stamp);
  c.sequence++;
  c.enabled = true;
  c.released = false;
  c.source_ns = c.receipt_ns = ++stamp;
  manager.accept(c, stamp);
  result = manager.update(s, stamp, .001);
  EXPECT_FALSE(result.control.motion_enabled);
  EXPECT_EQ(law.calls, 0);
  home_input(false, false);
  for (int n = 0; n < 1000; ++n) {
    stamp += 1000000;
    home_input(false, false);
    result = manager.update(s, stamp, .001);
    if (!result.homing.active)
      break;
  }
  EXPECT_EQ(result.homing.phase, ft::HomePhase::cancelled);
  EXPECT_FALSE(result.control.motion_enabled);
  EXPECT_EQ(law.calls, 0);
  c.sequence++;
  c.source_ns = c.receipt_ns = ++stamp;
  manager.accept(c, stamp);
  EXPECT_FALSE(manager.update(s, stamp, .001).control.motion_enabled);
  c.sequence++;
  c.enabled = false;
  c.released = true;
  c.source_ns = c.receipt_ns = ++stamp;
  manager.accept(c, stamp);
  result = manager.update(s, stamp, .001);
  EXPECT_TRUE(result.control.position_hold_active);
  EXPECT_EQ(result.control.reason, ft::Reason::position_hold);
  EXPECT_TRUE(result.control.nominal.isZero());
  c.sequence++;
  c.enabled = true;
  c.released = false;
  c.source_ns = c.receipt_ns = ++stamp;
  manager.accept(c, stamp);
  EXPECT_TRUE(manager.update(s, stamp, .001).control.motion_enabled);
  EXPECT_EQ(law.calls, 1);
}

TEST(Manager, InvalidLawResultAndZeroPeriodTerminateBeforeSafetyHandoff) {
  ProbeController law;
  ft::DampingController damping(20, 2.5);
  ft::PositionHoldController hold({});
  ft::ControllerManager manager({}, {}, law, hold, damping);
  EXPECT_EQ(manager.update(state(), now, 0).control.state_fault, ft::StateFault::period);
  manager.accept(command(1, false, true), now + 1);
  manager.accept(command(2, true), now + 2);
  law.result.fault = ft::StateFault::invalid_reference;
  auto out = manager.update(state(), now + 2, .001).control;
  EXPECT_EQ(out.state_fault, ft::StateFault::invalid_reference);
  EXPECT_FALSE(out.motion_enabled);
  manager.accept(command(3, false, true), now + 3);
  manager.accept(command(4, true), now + 4);
  law.result.fault = ft::StateFault::none;
  law.result.torque[0] = std::numeric_limits<double>::quiet_NaN();
  out = manager.update(state(), now + 4, .001).control;
  EXPECT_EQ(out.state_fault, ft::StateFault::nonfinite_nominal);
  EXPECT_FALSE(out.motion_enabled);
}

TEST(TorqueController, IncompatibleReferenceIsAnExplicitFault) {
  ft::CartesianPdController cartesian({});
  ft::HomingController homing({});
  ft::PositionHoldController hold({});
  EXPECT_EQ(cartesian.compute(state(), ft::JointReference{}, .001).fault,
            ft::StateFault::invalid_reference);
  EXPECT_EQ(homing.compute(state(), ft::CartesianReference{}, .001).fault,
            ft::StateFault::invalid_reference);
  EXPECT_EQ(hold.compute(state(), ft::CartesianReference{}, .001).fault,
            ft::StateFault::invalid_reference);
}

TEST(Manager, ReleaseCapturesMeasuredJointsOnceAndHoldsTheUnobservedSeventhJoint) {
  Fixture core;
  arm(core);
  auto s = state();
  s.q[6] = .3;
  auto c = command(3, true);
  c.position.x() += .1;
  core.accept(c, now + 3);
  ASSERT_TRUE(core.update(s, now + 3, .001).motion_enabled);
  core.accept(command(4, false, true), now + 4);
  auto out = core.update(s, now + 4, .001);
  ASSERT_TRUE(out.position_hold_active);
  EXPECT_FALSE(out.motion_enabled);
  EXPECT_EQ(out.reason, ft::Reason::position_hold);
  EXPECT_TRUE(out.nominal.isZero());  // Does not pursue the old Quest goal.
  // Keep release commands fresh throughout the stiffness ramp. Disturb only
  // the redundant joint, which Cartesian-only stiffness would not restore.
  s.q[6] += .02;
  uint64_t stamp = now + 4;
  for (int n = 0; n < 300; ++n) {
    stamp += 1000000;
    c = command(5 + n, false, true);
    c.source_ns = c.receipt_ns = stamp;
    core.accept(c, stamp);
    out = core.update(s, stamp, .001);
  }
  EXPECT_TRUE(out.position_hold_active);
  EXPECT_NEAR(out.nominal[6], -75 * .02, 1e-12);
  // Further displacement produces additional restoring torque, not a new anchor.
  s.q[6] += .01;
  EXPECT_NEAR(core.update(s, stamp + 1, .001).nominal[6], -75 * .03, 1e-12);
}

TEST(Manager, InputFaultsPreserveTheHoldAndFreshReleaseCannotRecaptureIt) {
  Fixture core;
  auto s = state();
  arm(core);
  ASSERT_TRUE(core.update(s, now + 2, .001).motion_enabled);
  core.accept(command(3, false, true), now + 3);
  ASSERT_TRUE(core.update(s, now + 3, .001).position_hold_active);
  s.q[0] = .1;
  auto out = core.update(s, now + 100000004, .001);
  EXPECT_EQ(out.reason, ft::Reason::position_hold);
  EXPECT_TRUE(out.position_hold_active);
  EXPECT_LT(out.nominal[0], 0);  // Restore the parked configuration.
  auto c = command(4, true);
  c.source_ns = c.receipt_ns = now + 100000005;
  core.accept(c, c.source_ns);
  EXPECT_FALSE(core.update(s, c.source_ns, .001).motion_enabled);
  c.sequence++;
  c.source_ns++;
  c.receipt_ns++;
  c.enabled = false;
  c.released = true;
  core.accept(c, c.source_ns);
  out = core.update(s, c.source_ns, .001);
  EXPECT_TRUE(out.position_hold_active);
  EXPECT_LT(out.nominal[0], 0);  // Fresh release cannot follow displaced joints.
  core.manager.fault(ft::Reason::invalid_command);
  core.manager.fault(ft::Reason::homing);  // Arbitration cannot erase the fault gate.
  EXPECT_FALSE(core.update(s, c.source_ns + 1, .001).position_hold_active);
  core.manager.shutdown();
  EXPECT_FALSE(core.update(s, c.source_ns + 2, .001).position_hold_active);
}

TEST(Manager, HoldingRecapturesAfterTeleoperationAndReturnsItsOwnership) {
  ProbeController law;
  ProbeController hold;
  ft::DampingController damping(20, 2.5);
  ft::ControllerManager manager({}, {}, law, hold, damping);
  auto s = state();
  EXPECT_TRUE(manager.update(s, now, .001).control.position_hold_active);
  EXPECT_EQ(hold.resets, 1);
  manager.accept(command(1, false, true), now + 1);
  manager.update(s, now + 1, .001);
  EXPECT_EQ(hold.resets, 1);  // Quest readiness cannot move the startup anchor.
  manager.update(s, now + 2, .001);
  EXPECT_EQ(hold.resets, 1);
  manager.accept(command(2, true), now + 2);
  auto out = manager.update(s, now + 2, .001).control;
  EXPECT_TRUE(out.motion_enabled);
  EXPECT_FALSE(out.position_hold_active);
  EXPECT_EQ(law.calls, 1);
  manager.accept(command(3, false, true), now + 3);
  EXPECT_TRUE(manager.update(s, now + 3, .001).control.position_hold_active);
  EXPECT_EQ(hold.resets, 2);
}

TEST(PositionHold, StiffnessRampsAndJointDampingAndCoriolisArePreserved) {
  ft::PositionHoldParameters p;
  ASSERT_TRUE(p.valid());
  ft::PositionHoldController hold(p);
  auto s = state();
  s.coriolis.setConstant(1);
  s.dq.setConstant(.02);
  ft::JointReference reference{s.q + ft::Vector7::Constant(.01), ft::Vector7::Zero()};
  hold.reset(s);
  auto first = hold.compute(s, reference, .001);
  const ft::Vector7 damping = -p.damping.cwiseProduct(s.dq) + s.coriolis;
  EXPECT_LT((first.torque - damping).norm(), .001);
  ft::NominalTorque out;
  for (int i = 0; i < 300; ++i)
    out = hold.compute(s, reference, .001);
  EXPECT_TRUE(out.torque.isApprox(.01 * p.stiffness + damping));
  hold.reset(s);
  EXPECT_TRUE(hold.compute(s, reference, .001).torque.isApprox(first.torque));
  EXPECT_EQ(hold.compute(s, reference, 0).fault, ft::StateFault::period);
}

TEST(PositionHold, InvalidParametersAndReferencesAreRejected) {
  ft::PositionHoldParameters p;
  p.stiffness[0] = 0;
  EXPECT_FALSE(p.valid());
  p = {};
  p.damping[6] = -1;
  EXPECT_FALSE(p.valid());
  p = {};
  p.stiffness_ramp_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(p.valid());
  p = {};
  p.stiffness_ramp_seconds = 0;
  EXPECT_FALSE(p.valid());
  ft::PositionHoldController hold({});
  ft::JointReference reference;
  reference.position[0] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(hold.compute(state(), reference, .001).fault, ft::StateFault::invalid_reference);
}

TEST(Manager, StartupHoldSurvivesMissingInvalidAndStaleQuestInputWithoutFollowingDrift) {
  Fixture core;
  auto s = state();
  auto out = core.update(s, now, .001);
  ASSERT_TRUE(out.position_hold_active);
  // The mapper watchdog emits invalid disarmed commands until Quest is ready.
  // Disturb all seven joints while no operator has ever requested movement.
  const ft::Vector7 displacement = ft::Vector7::Constant(.01);
  s.q += displacement;
  s.position.x() += .01;
  for (int n = 1; n <= 400; ++n) {
    auto c = command(n, false);
    c.valid = false;
    c.source_ns = c.receipt_ns = now + n * 1000000;
    core.accept(c, c.receipt_ns);
    out = core.update(s, c.receipt_ns, .001);
    ASSERT_TRUE(out.position_hold_active) << n;
    EXPECT_FALSE(out.motion_enabled);
  }
  const ft::PositionHoldParameters gains;
  EXPECT_TRUE(out.nominal.isApprox(-gains.stiffness.cwiseProduct(displacement)));
  EXPECT_NEAR(out.target_position.x(), .4, 1e-12);
  auto released = command(401, false, true);
  released.source_ns = released.receipt_ns = now + 401000000;
  core.accept(released, released.receipt_ns);
  out = core.update(s, released.receipt_ns, .001);
  EXPECT_TRUE(out.nominal.isApprox(-gains.stiffness.cwiseProduct(displacement)));
  // Repeated stale/fault processing must neither recapture q nor restart ramp.
  for (int n = 1; n < 50; ++n) {
    out = core.update(s, released.receipt_ns + 200000000 + n * 1000000, .001);
    ASSERT_TRUE(out.position_hold_active);
    EXPECT_TRUE(out.nominal.isApprox(-gains.stiffness.cwiseProduct(displacement)));
  }
}

TEST(Manager, StartupSessionAndQueueFaultsCannotMoveTheLocalHoldAnchor) {
  ProbeController law, hold;
  ft::DampingController damping(20, 2.5);
  ft::ControllerManager manager({}, {}, law, hold, damping);
  auto s = state();
  manager.update(s, now, .001);
  ASSERT_EQ(hold.resets, 1);
  const ft::Reason reasons[] = {ft::Reason::invalid_command, ft::Reason::stale, ft::Reason::order,
                                ft::Reason::session_change, ft::Reason::queue_overflow};
  for (auto reason : reasons) {
    manager.fault(reason);
    EXPECT_TRUE(manager.update(s, now + 1, .001).control.position_hold_active);
    EXPECT_EQ(hold.resets, 1);
  }
  manager.accept(command(1, true), now + 1);  // Startup-held trigger is not permission.
  EXPECT_FALSE(manager.update(s, now + 1, .001).control.motion_enabled);
  EXPECT_EQ(hold.resets, 1);
  manager.shutdown();
  EXPECT_FALSE(manager.update(s, now + 2, .001).control.position_hold_active);
}
TEST(Manager, PausedHoldSurvivesTrackingFaultAndStalenessWithoutRecapture) {
  Fixture f;
  auto s = state();
  arm(f);
  f.update(s, now + 2, .001);
  f.accept(command(3, false, true), now + 3);
  f.update(s, now + 3, .001);
  s.q[6] = .02;
  for (int i = 0; i < 400; ++i) {
    auto c = command(4 + i, false);
    c.valid = false;
    c.source_ns = c.receipt_ns = now + 4000000 + i * 1000000;
    f.accept(c, c.receipt_ns);
    const auto out = f.update(s, c.receipt_ns, .001);
    ASSERT_TRUE(out.position_hold_active);
    EXPECT_FALSE(out.motion_enabled);
  }
  EXPECT_NEAR(f.update(s, now + 500000000, .001).nominal[6], -1.5, 1e-10);
}

TEST(Manager, ReferenceSpeedAccelerationAndTwistStayCoherentAcrossReversals) {
  ft::ManagerParameters limits;
  limits.reference_shaping_enabled = true;
  Fixture f(limits);
  arm(f);
  auto measured = state();
  auto previous = f.update(measured, now + 2, .001);
  for (int i = 1; i <= 1000; ++i) {
    const uint64_t stamp = now + 2 + i * 1000000ULL;
    auto c = command(i + 2, true);
    c.source_ns = c.receipt_ns = stamp;
    c.position.x() += i < 500 ? .02 : -.02;
    c.orientation = Eigen::AngleAxisd(i < 500 ? .1 : -.1, Eigen::Vector3d::UnitZ());
    c.twist.x() = i < 500 ? .2 : -.2;
    f.accept(c, stamp);
    const auto out = f.update(measured, stamp, .001);
    ASSERT_TRUE(out.motion_enabled) << i;
    EXPECT_LE(out.desired_twist.head<3>().norm(), limits.reference_linear_speed + 1e-12);
    EXPECT_LE(out.desired_twist.tail<3>().norm(), limits.reference_angular_speed + 1e-12);
    EXPECT_LE((out.desired_twist.head<3>() - previous.desired_twist.head<3>()).norm(),
              limits.reference_linear_acceleration * .001 + 1e-12);
    EXPECT_LE((out.desired_twist.tail<3>() - previous.desired_twist.tail<3>()).norm(),
              limits.reference_angular_acceleration * .001 + 1e-12);
    EXPECT_TRUE(((out.target_position - previous.target_position) / .001)
                    .isApprox(out.desired_twist.head<3>(), 1e-7));
    Eigen::AngleAxisd delta(out.target_orientation * previous.target_orientation.conjugate());
    EXPECT_LT((delta.axis() * delta.angle() / .001 - out.desired_twist.tail<3>()).norm(), 1e-9);
    measured.position = out.target_position;
    measured.orientation = out.target_orientation;
    previous = out;
  }
}

TEST(Manager, ImmobilePlantCannotWindUpAReferenceWithoutLimit) {
  ft::ManagerParameters p;
  p.reference_shaping_enabled = p.tracking_protection_enabled = true;
  Fixture f(p);
  arm(f);
  auto measured = state();
  bool stopped = false;
  for (int i = 1; i <= 2000; ++i) {
    const auto stamp = now + i * 1000000ULL;
    auto c = command(i + 2, true);
    c.source_ns = c.receipt_ns = stamp;
    c.position.x() += .1;
    f.accept(c, stamp);
    const auto out = f.update(measured, stamp, .001);
    if (out.protective_stop) {
      EXPECT_EQ(out.reason, ft::Reason::tracking_limit);
      stopped = true;
      break;
    }
    EXPECT_LT((out.target_position - measured.position).norm(), .03);
  }
  EXPECT_TRUE(stopped);
}

TEST(Manager, OptionalLimitsDefaultOffAndMappedReferencePassesThrough) {
  const ft::ManagerParameters p;
  EXPECT_FALSE(p.reference_shaping_enabled);
  EXPECT_FALSE(p.tracking_protection_enabled);
  Fixture f(p);
  arm(f);
  const auto measured = state();
  for (int i = 1; i <= 600; ++i) {
    auto c = command(i + 2, true);
    c.source_ns = c.receipt_ns = now + i * 1000000ULL;
    c.position.x() += .1;
    c.orientation = Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitZ());
    c.twist << .4, 0, 0, 0, 0, 2.;
    f.accept(c, c.receipt_ns);
    const auto out = f.update(measured, c.receipt_ns, .001);
    ASSERT_TRUE(out.motion_enabled);
    EXPECT_FALSE(out.protective_stop);
    EXPECT_TRUE(out.target_position.isApprox(c.position));
    EXPECT_TRUE(out.target_orientation.isApprox(c.orientation));
    EXPECT_TRUE(out.desired_twist.isApprox(c.twist));
    EXPECT_GT(out.nominal.head<3>().norm(), 10.);
    EXPECT_GT(out.nominal.segment<3>(3).norm(), 1.5);
  }
}

TEST(Manager, TrackingProtectionCanBeEnabledWithoutReferenceShaping) {
  for (int scenario = 0; scenario < 3; ++scenario) {
    SCOPED_TRACE(scenario);
    ft::ManagerParameters p;
    p.tracking_protection_enabled = true;
    Fixture f(p);
    arm(f);
    bool stopped = false;
    for (int i = 1; i <= 300; ++i) {
      auto c = command(i + 2, true);
      c.source_ns = c.receipt_ns = now + i * 1000000ULL;
      if (scenario == 0)
        c.position.x() += .04;
      else if (scenario == 1)
        c.orientation = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ());
      else
        c.position.x() += .02;  // More than half the threshold for 250 ms.
      f.accept(c, c.receipt_ns);
      const auto out = f.update(state(), c.receipt_ns, .001);
      if (out.protective_stop) {
        EXPECT_EQ(out.reason, ft::Reason::tracking_limit);
        EXPECT_FALSE(out.motion_enabled);
        if (scenario == 2) {
          EXPECT_GE(i, 250);
          EXPECT_LE(i, 251);
        } else {
          EXPECT_EQ(i, 1);
        }
        stopped = true;
        break;
      }
      ASSERT_TRUE(out.motion_enabled);
    }
    EXPECT_TRUE(stopped);
  }
}

TEST(Manager, OptionalReferenceProjectionKeepsBoundarySlidingAndRetreatConnected) {
  Fixture core;
  core.manager.reference_projection(
      [](const void*, const ft::Vector6& input) noexcept -> ft::Vector6 {
        auto result = input;
        result[0] = std::max(0., result[0]);
        return result;
      },
      nullptr);
  arm(core);
  auto measured = state();
  auto inward = command(3, true);
  inward.position << .35, .02, .4;
  inward.twist << -.1, .2, 0, 0, 0, 0;
  core.accept(inward, now + 3);
  const auto sliding = core.update(measured, now + 3, .001);
  EXPECT_TRUE(sliding.motion_enabled);
  EXPECT_FALSE(sliding.protective_stop);
  EXPECT_NEAR(sliding.target_position.x(), .4, 1e-12);
  EXPECT_NEAR(sliding.target_position.y(), .02, 1e-12);
  EXPECT_NEAR(sliding.desired_twist.x(), 0, 1e-12);
  EXPECT_NEAR(sliding.desired_twist.y(), .2, 1e-12);
  auto retreat = command(4, true);
  retreat.position << .43, .03, .4;
  core.accept(retreat, now + 4);
  const auto outward = core.update(measured, now + 4, .001);
  EXPECT_TRUE(outward.motion_enabled);
  EXPECT_NEAR(outward.target_position.x(), .48, 1e-12);  // .08 retreat from the held .4 anchor.
  EXPECT_NEAR(outward.target_position.y(), .03, 1e-12);
}

TEST(Manager, BlockedInputDoesNotAccumulateAndSmallRetreatWorksWithEitherShapingMode) {
  for (bool shaping : {false, true}) {
    ft::ManagerParameters parameters;
    parameters.reference_shaping_enabled = shaping;
    Fixture core(parameters);
    core.manager.reference_projection(
        [](const void*, const ft::Vector6& input) noexcept -> ft::Vector6 {
          auto result = input;
          result[2] = std::max(0., result[2]);
          return result;
        },
        nullptr);
    arm(core);
    auto measured = state();
    auto request = command(3, true);
    request.position.z() = .25;
    request.twist[2] = -.05;
    core.accept(request, now + 3);
    for (int i = 0; i < 30; ++i) {
      const auto held = core.update(measured, now + 3 + i, .001);
      EXPECT_TRUE(held.motion_enabled);
      EXPECT_NEAR(held.target_position.z(), .4, 1e-12);
      EXPECT_NEAR(held.desired_twist[2], 0, 1e-12);
    }
    request = command(4, true);
    request.position.z() = .251;   // Still far beyond the held boundary, but moving OUT.
    request.position.x() += .001;  // Tangential movement remains available.
    request.twist[2] = .01;
    core.accept(request, now + 40);
    const auto retreat = core.update(measured, now + 40, .001);
    EXPECT_TRUE(retreat.motion_enabled);
    EXPECT_GT(retreat.target_position.z(), .4);
    EXPECT_GT(retreat.target_position.x(), .4);
    EXPECT_GT(retreat.desired_twist[2], 0);
  }
}

TEST(Manager, RotationalBoundaryHoldsAcceptedOrientationAndAllowsSmallReversal) {
  Fixture core;
  core.manager.reference_projection(
      [](const void*, const ft::Vector6& input) noexcept -> ft::Vector6 {
        auto result = input;
        result[5] = std::max(0., result[5]);
        return result;
      },
      nullptr);
  arm(core);
  auto measured = state();
  auto request = command(3, true);
  request.orientation = Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitZ());
  request.twist[5] = -.05;
  core.accept(request, now + 3);
  const auto held = core.update(measured, now + 3, .001);
  EXPECT_TRUE(held.target_orientation.isApprox(measured.orientation, 1e-12));
  EXPECT_NEAR(held.desired_twist[5], 0, 1e-12);
  request.sequence = 4;
  request.receipt_ns = request.source_ns = now + 4;
  request.orientation = Eigen::AngleAxisd(-.199, Eigen::Vector3d::UnitZ());
  request.twist[5] = .01;
  core.accept(request, now + 4);
  const auto retreat = core.update(measured, now + 4, .001);
  EXPECT_NEAR(Eigen::AngleAxisd(retreat.target_orientation).angle(), .001, 1e-12);
  EXPECT_GT(retreat.desired_twist[5], 0);
  core.accept(command(5, false, true), now + 5);
  core.update(measured, now + 5, .001);
  core.accept(command(6, true), now + 6);
  const auto reengaged = core.update(measured, now + 6, .001);
  EXPECT_TRUE(reengaged.target_orientation.isApprox(measured.orientation, 1e-12));
}

TEST(Manager, CbfFailureLatchesProtectiveBrakingAndHold) {
  Fixture core;
  auto measured = state();
  measured.dq[0] = .1;
  core.manager.request_protective_stop(ft::Reason::cbf_failure);
  const auto stopped = core.update(measured, now + 3, .001);
  EXPECT_TRUE(stopped.protective_stop);
  EXPECT_FALSE(stopped.motion_enabled);
  EXPECT_EQ(stopped.reason, ft::Reason::cbf_failure);
  EXPECT_TRUE(stopped.nominal.allFinite());
}

TEST(Manager, ConstraintPreparationSeesReleaseAndEngagementBeforeReferenceProjection) {
  Fixture core;
  auto s = state();
  struct Context {
    int preparations{0}, projections{0};
    bool movable{false};
  } context;
  const auto prepare = [](void* opaque, bool movable) noexcept {
    auto& c = *static_cast<Context*>(opaque);
    ++c.preparations;
    c.movable = movable;
  };
  core.manager.reference_projection(
      [](const void* opaque, const ft::Vector6& step) noexcept -> ft::Vector6 {
        auto& c = *const_cast<Context*>(static_cast<const Context*>(opaque));
        EXPECT_TRUE(c.movable);
        EXPECT_GT(c.preparations, 0);
        ++c.projections;
        return step;
      },
      &context);
  auto output = core.manager.update(s, now, .001, prepare, &context);
  EXPECT_EQ(context.preparations, 1);
  EXPECT_FALSE(context.movable);
  EXPECT_TRUE(output.control.position_hold_active);
  arm(core);
  output = core.manager.update(s, now + 2, .001, prepare, &context);
  EXPECT_EQ(context.preparations, 2);
  EXPECT_TRUE(context.movable);
  EXPECT_TRUE(output.control.motion_enabled);
  EXPECT_GT(context.projections, 0);
  const int projections = context.projections;
  core.accept(command(3, false, true), now + 3);
  output = core.manager.update(s, now + 3, .001, prepare, &context);
  EXPECT_EQ(context.preparations, 3);
  EXPECT_FALSE(context.movable);
  EXPECT_TRUE(output.control.position_hold_active);
  EXPECT_EQ(context.projections, projections);
}

TEST(Manager, BindingConstraintDiscardsStoredTrackingErrorAndRetreatsWithoutUnwinding) {
  Fixture core;
  bool blocked = false;
  core.manager.reference_projection(
      [](const void* opaque, const ft::Vector6& input) noexcept -> ft::Vector6 {
        auto out = input;
        if (*static_cast<const bool*>(opaque) && out[0] > 0)
          out[0] = 0;
        return out;
      },
      &blocked, true);
  arm(core);
  const auto measured = state();
  auto request = command(3, true);
  request.position.x() = .41;
  core.accept(request, now + 3);
  EXPECT_NEAR(core.update(measured, now + 3, .001).target_position.x(), .41, 1e-12);
  blocked = true;
  request = command(4, true);
  request.position.x() = .42;
  core.accept(request, now + 4);
  // Discard the existing 10 mm inward tracking lead, as well as the new input.
  EXPECT_NEAR(core.update(measured, now + 4, .001).target_position.x(), .4, 1e-12);
  request = command(5, true);
  request.position.x() = .4199;
  core.accept(request, now + 5);
  // A 0.1 mm reversal acts immediately; it does not unwind the blocked 20 mm.
  EXPECT_NEAR(core.update(measured, now + 5, .001).target_position.x(), .3999, 1e-12);
}
