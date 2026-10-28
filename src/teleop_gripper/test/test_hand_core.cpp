#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "teleop_gripper/hand_core.hpp"
namespace ft = teleop_gripper;
namespace {
struct Fixture {
  static ft::HandParameters parameters() {
    ft::HandParameters p;
    p.grasp_force = 5;
    return p;
  }
  ft::HandCore core{parameters()};
  uint64_t now{1000000000}, sequence{0}, generation{1};
  const char* session{"first"};
  ft::HandRequest input(double value, bool valid = true, bool active = true,
                        uint64_t interval = 10000000) {
    now += interval;
    ft::GripInput c;
    std::strcpy(c.session.data(), session);
    c.sequence = ++sequence;
    c.generation = generation;
    c.source_ns = now;
    c.value = value;
    c.valid = valid;
    c.joystick_active = active;
    return core.input(c, now);
  }
  void calibrate() {
    ASSERT_EQ(core.calibrate(true, true).operation, ft::HandOperation::calibrate);
    core.complete(ft::HandOperation::calibrate, true, true, .08, .08, now);
    ASSERT_TRUE(core.calibrated());
  }
  void stopped(ft::HandOperation operation = ft::HandOperation::close) {
    core.complete(operation, true, true, .03, .08, now);
  }
  void hold() {
    calibrate();
    input(0);
    ASSERT_EQ(input(.8).operation, ft::HandOperation::close);
    core.complete(ft::HandOperation::close, true, true, .005, .08, now, true);
    ASSERT_TRUE(core.grasp_confirmed());
  }
};
}  // namespace
TEST(Hand, MirroredMappingIsInControllerCoordinates) {
  EXPECT_DOUBLE_EQ(ft::inward_joystick(-.8, true), .8);
  EXPECT_DOUBLE_EQ(ft::inward_joystick(.8, true), -.8);
  EXPECT_DOUBLE_EQ(ft::inward_joystick(.8, false), .8);
  EXPECT_DOUBLE_EQ(ft::inward_joystick(-.8, false), -.8);
}
TEST(Hand, CalibrationNeedsBothConfirmationsAndNeutral) {
  Fixture f;
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.core.calibrate(false, true).operation, ft::HandOperation::none);
  EXPECT_EQ(f.core.calibrate(true, false).operation, ft::HandOperation::none);
  f.calibrate();
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(0, true, false).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  f.input(0);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::close);
}
TEST(Hand, ForceIsExplicitlyConfiguredAndOpeningRemainsAvailable) {
  Fixture f;
  f.core = ft::HandCore(ft::HandParameters{});
  f.calibrate();
  f.input(0);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_NE(std::string(f.core.message()).find("grasp_force_n"), std::string::npos);
  f.input(0);
  EXPECT_EQ(f.input(-.8).operation, ft::HandOperation::open);
}
TEST(Hand, DirectionSelectsFixedSpeedRegardlessOfDeflection) {
  for (double axis : {.41, .65, 1.}) {
    Fixture f;
    f.calibrate();
    f.input(0);
    const auto r = f.input(axis);
    ASSERT_EQ(r.operation, ft::HandOperation::close);
    EXPECT_DOUBLE_EQ(r.speed, .02);
    EXPECT_DOUBLE_EQ(r.width, 0);
    EXPECT_DOUBLE_EQ(r.force, 5);
    EXPECT_DOUBLE_EQ(r.minimum_grasp_width, .003);
    EXPECT_DOUBLE_EQ(r.maximum_grasp_width, .007);
    EXPECT_GT(r.epsilon_outer, .007);
  }
}
TEST(Hand, NeutralStopsBothClosingAndOpeningWithoutAutomaticCompletion) {
  for (double axis : {-.8, .8}) {
    Fixture f;
    f.calibrate();
    f.input(0);
    const auto operation = axis > 0 ? ft::HandOperation::close : ft::HandOperation::open;
    ASSERT_EQ(f.input(axis).operation, operation);
    EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
    EXPECT_EQ(f.input(0).operation, ft::HandOperation::none);
    f.stopped(operation);
    EXPECT_EQ(f.core.state(), ft::HandState::stopped);
    for (int n = 0; n < 20; ++n)
      EXPECT_EQ(f.input(0).operation, ft::HandOperation::none);
  }
}
TEST(Hand, HysteresisSuppressesChatterAndOppositeDirectionCanInterrupt) {
  Fixture f;
  f.calibrate();
  f.input(0);
  EXPECT_EQ(f.input(.3).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(.5).operation, ft::HandOperation::close);
  EXPECT_EQ(f.input(.3).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(-.8).operation, ft::HandOperation::stop);
  auto next = f.core.complete(ft::HandOperation::close, true, true, .03, .08, f.now);
  EXPECT_EQ(next.operation, ft::HandOperation::open);
  EXPECT_DOUBLE_EQ(next.speed, .1);
}
TEST(Hand, OnlyLatestFreshDirectionMayResumeAfterStop) {
  for (bool neutral : {false, true}) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.input(.8);
    f.input(-.8);
    if (neutral)
      f.input(0);
    const auto next = f.core.complete(ft::HandOperation::close, true, true, .03, .08, f.now);
    EXPECT_EQ(next.operation, neutral ? ft::HandOperation::none : ft::HandOperation::open);
  }
}
TEST(Hand, ConfirmedGraspSurvivesNeutralAndHeldInputWithoutRetries) {
  Fixture f;
  f.hold();
  EXPECT_DOUBLE_EQ(f.core.commanded_force(), 5);
  for (int n = 0; n < 100; ++n) {
    EXPECT_EQ(f.input(n < 50 ? .8 : 0).operation, ft::HandOperation::none);
    EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
  }
  EXPECT_TRUE(f.core.grasp_confirmed());
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(-.8).operation, ft::HandOperation::open);
  EXPECT_FALSE(f.core.grasp_confirmed());
}
TEST(Hand, CompletionRacePreservesGraspIfBackendSkippedAnUnnecessaryStop) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.input(.8);
  EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
  f.core.complete(ft::HandOperation::close, true, true, .005, .08, f.now, true);
  EXPECT_TRUE(f.core.grasp_confirmed());
  EXPECT_FALSE(f.core.busy());
}
TEST(Hand, EmptyClosureAndUnexpectedCableWidthsAreNotConfirmedGrasps) {
  for (double width : {0., .001, .012}) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.input(.8);
    f.core.complete(ft::HandOperation::close, false, true, width, .08, f.now, false);
    EXPECT_FALSE(f.core.grasp_confirmed());
    EXPECT_TRUE(f.core.release_required());
    for (int n = 0; n < 40; ++n) {
      EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
      EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
    }
  }
}
TEST(Hand, InputFaultStopsUnfinishedMotionButRetainsConfirmedHold) {
  for (bool holding : {false, true}) {
    for (int fault = 0; fault < 6; ++fault) {
      Fixture f;
      if (holding)
        f.hold();
      else {
        f.calibrate();
        f.input(0);
        f.input(.8);
      }
      ft::HandRequest result;
      if (fault == 0)
        result = f.input(.8, false);
      if (fault == 1)
        result = f.input(.8, true, false);
      if (fault == 2) {
        f.session = "replacement";
        result = f.input(.8);
      }
      if (fault == 3) {
        ++f.generation;
        result = f.input(.8);
      }
      if (fault == 4) {
        f.now += 200000000;
        result = f.core.watchdog(f.now);
      }
      if (fault == 5)
        result = f.input(.8, true, true, 200000000);
      EXPECT_EQ(result.operation, holding ? ft::HandOperation::none : ft::HandOperation::stop);
      if (!holding)
        f.stopped();
      EXPECT_EQ(f.core.grasp_confirmed(), holding);
      for (int n = 0; n < 4; ++n)
        EXPECT_EQ(f.input(-.8).operation, ft::HandOperation::none);
      f.input(0);
      EXPECT_EQ(f.input(-.8).operation, ft::HandOperation::open);
    }
  }
}
TEST(Hand, StaleDuplicateFutureNonfiniteAndUncertainSamplesCannotMove) {
  for (int fault = 0; fault < 6; ++fault) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.input(.8);
    ft::GripInput c;
    std::strcpy(c.session.data(), "first");
    c.valid = c.joystick_active = true;
    c.value = .8;
    c.sequence = f.sequence + 1;
    c.source_ns = f.now + 1;
    c.generation = 1;
    if (fault == 0)
      c.sequence = f.sequence;
    if (fault == 1)
      c.source_ns = f.now - 200000000;
    if (fault == 2)
      c.source_ns = f.now + 20000000;
    if (fault == 3)
      c.uncertainty_ns = 10000001;
    if (fault == 4)
      c.value = std::numeric_limits<double>::quiet_NaN();
    if (fault == 5)
      c.value = 1.1;
    EXPECT_EQ(f.core.input(c, f.now + 1).operation, ft::HandOperation::stop);
  }
}
TEST(Hand, FailureRequiresNeutralAndCommunicationFailureRequiresCalibration) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.input(.8);
  f.core.complete(ft::HandOperation::close, false, true, .01, .08, f.now);
  EXPECT_TRUE(f.core.calibrated());
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  f.input(0);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::close);
  f.core.complete(ft::HandOperation::close, false, false, .01, .08, f.now);
  EXPECT_FALSE(f.core.calibrated());
  f.input(0);
  EXPECT_EQ(f.input(-.8).operation, ft::HandOperation::none);
}
TEST(Hand, CalibrationIgnoresQuestLossAndShutdownPreservesAnIdleHold) {
  Fixture f;
  f.core.calibrate(true, true);
  EXPECT_EQ(f.input(.8, false).operation, ft::HandOperation::none);
  f.now += 200000000;
  EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
  Fixture holding;
  holding.hold();
  EXPECT_EQ(holding.core.shutdown().operation, ft::HandOperation::none);
  EXPECT_TRUE(holding.core.grasp_confirmed());
  EXPECT_EQ(holding.input(-.8).operation, ft::HandOperation::none);
}
TEST(Hand, ParametersAndCalibratedTravelAreValidated) {
  auto p = Fixture::parameters();
  ASSERT_TRUE(p.valid());
  p.neutral = p.engage;
  EXPECT_FALSE(p.valid());
  p.neutral = .2;
  p.cable_tolerance = .006;
  EXPECT_FALSE(p.valid());
  p.cable_tolerance = .002;
  p.grasp_force = 0;
  EXPECT_FALSE(p.valid());
  p.grasp_force = 5;
  p.open_speed = 0;
  EXPECT_FALSE(p.valid());
  p.open_speed = .1;
  p.open_width = .09;
  Fixture f;
  f.core = ft::HandCore(p);
  f.calibrate();
  f.input(0);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
}
