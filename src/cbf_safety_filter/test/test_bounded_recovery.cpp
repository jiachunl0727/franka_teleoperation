#include <gtest/gtest.h>

#include "cbf_safety_filter/recovery_projection.hpp"
#include "cbf_safety_filter/safety_filter.hpp"
#include "fixture.hpp"
namespace cbf = cbf_safety_filter;
class BoundedRecovery : public ::testing::TestWithParam<cbf::RecoveryTorqueMethod> {};
namespace {
cbf::Configuration config() {
  cbf::Configuration c;
  c.inter_arm.enabled = true;
  c.backend = cbf::Backend::qpoases;
  c.recovery_torque_method = BoundedRecovery::GetParam();
  c.inter_arm.clearance = .01;
  c.inter_arm.alpha1 = c.inter_arm.alpha2 = 10;
  c.peer_max_age_us = c.peer_max_skew_us = 10000;
  c.max_constraints = 512;
  c.max_filter_time_us = c.max_model_age_us = 1000000;
  cbf::BodySphere b;
  b.frame = 0;
  b.radius = .04;
  c.body_spheres = {b};
  return c;
}
cbf_test::Model plant(double x, double v = 0) {
  cbf_test::Model m;
  m.dq[0] = v;
  for (int f = 0; f < cbf::kFrames; ++f) {
    m.poses[f].setIdentity();
    m.poses[f].translation().x() = x;
    m.jacobians[f].setZero();
    m.jacobians[f](0, 0) = 1;
    m.jacobians[f](2, 1) = 1;
  }
  return m;
}
cbf::PeerSnapshot peer() {
  cbf::PeerSnapshot p;
  p.valid = true;
  p.timestamp_ns = 1;
  p.count = 1;
  p.speed_limit = 1;
  p.acceleration_limit = 5;
  p.spheres[0] = {{0, 0, 0}, {0, 0, 0}, .04, 1};
  return p;
}
const cbf::TorqueBounds box{cbf::Vec7::Constant(-1), cbf::Vec7::Constant(1)};
}  // namespace
TEST_P(BoundedRecovery, PeerPositionsAreNotPredictedOrExtrapolated) {
  auto c = config();
  auto m = plant(.095);
  auto p = peer();
  p.spheres[0].velocity.x() = .5;
  cbf::ConstraintBuilder builder(c);
  auto a = builder.build(m.view(), .001, 5000001, &box, &p);
  ASSERT_EQ(a.status, cbf::Status::solved_hard);
  p.spheres[0].velocity.x() = -.5;
  auto b = builder.build(m.view(), .001, 1, &box, &p);
  ASSERT_EQ(a.count, b.count);
  bool collision = false;
  for (int i = 0; i < a.count; ++i) {
    EXPECT_NEAR(a.rows[i].bound, b.rows[i].bound, 1e-12);
    if (a.rows[i].family == cbf::Family::inter_arm) {
      collision = true;
      EXPECT_NEAR(a.rows[i].h, .095 * .095 - .09 * .09, 1e-12);
    }
  }
  EXPECT_TRUE(collision);
  EXPECT_EQ(builder.build(m.view(), .001, 10000002, &box, &p).status, cbf::Status::stale_peer);
}
TEST_P(BoundedRecovery, ViolatedClearanceRejectsWorseningAndAcceptsRetreatWithoutAutoPush) {
  auto c = config();
  auto m = plant(.085);
  auto p = peer();
  cbf::SafetyFilter filter(c);
  for (double requested : {-.2, 0., .2}) {
    cbf::Vec7 nominal = cbf::Vec7::Zero();
    nominal[0] = requested;
    const auto r = filter.compute(m.view(), nominal, box, .001, 1, &p);
    ASSERT_TRUE(r.accepted()) << cbf::name(r.status);
    EXPECT_EQ(r.status, cbf::Status::verified_recovery);
    EXPECT_TRUE(filter.verify_command(r.torque));
    EXPECT_NEAR(r.torque[0], std::max(0., requested), 1e-6);
    EXPECT_GT(r.outside_domain_rows, 0);
  }
  ASSERT_EQ(filter.prepare(m.view(), box, .001, 1, &p, nullptr, false), cbf::Status::solved_hard);
  auto nominal = cbf::Vec7::Constant(.05);
  const auto held = filter.solve_prepared(nominal);
  ASSERT_TRUE(held.accepted());
  EXPECT_TRUE(held.torque.isApprox(nominal, 1e-12));
}
TEST_P(BoundedRecovery, InertiaUsesVerifiedBrakingAnchorInsteadOfAnInfeasibleStop) {
  auto c = config();
  auto m = plant(.085, -.2);
  auto p = peer();
  cbf::SafetyFilter filter(c);
  auto constrained = box;
  constrained.lower[0] = -.85;
  constrained.upper[0] = -.15;
  const auto r = filter.compute(m.view(), cbf::Vec7::Constant(-.5), constrained, .001, 1, &p);
  ASSERT_TRUE(r.accepted());
  EXPECT_TRUE(filter.verify_command(r.torque));
  EXPECT_NEAR(r.torque[0], -.15, 1e-6);
  EXPECT_GT(r.recovery_rows, 0);
  EXPECT_GT(r.max_requested_violation, 0);
}
TEST_P(BoundedRecovery, PositionAndVelocityViolationsAllowCorrectiveCommands) {
  for (bool position : {false, true}) {
    auto c = config();
    c.inter_arm.enabled = false;
    c.body_spheres.clear();
    c.position.enabled = position;
    c.velocity.enabled = !position;
    auto m = plant(.2);
    if (position)
      m.q[0] = 2.85;
    else
      m.dq[0] = 2.1;
    cbf::SafetyFilter filter(c);
    for (double requested : {-.2, .2}) {
      cbf::Vec7 nominal = cbf::Vec7::Zero();
      nominal[0] = requested;
      const auto r = filter.compute(m.view(), nominal, box, .001, 1);
      ASSERT_TRUE(r.accepted());
      EXPECT_TRUE(filter.verify_command(r.torque));
      EXPECT_LE(r.torque[0], 1e-6);
      if (requested < 0)
        EXPECT_NEAR(r.torque[0], requested, 1e-6);
    }
  }
}
TEST_P(BoundedRecovery, TableAndObstacleViolationsDoNotTerminateRecovery) {
  for (bool table : {false, true}) {
    auto c = config();
    c.inter_arm.enabled = false;
    c.body_spheres.clear();
    if (table) {
      c.table.enabled = true;
      c.table.height = .01;
      c.capsules.push_back({0, {0, 0, 0}, {0, 0, 0}, .04});
    } else {
      c.obstacles.enabled = true;
      c.obstacles.clearance = .01;
      c.points.push_back({0, {0, 0, 0}, .04});
      c.obstacle_spheres.push_back({{0, 0, 0}, .04});
      c.pairs.push_back({0, 0});
    }
    auto m = plant(.085);
    cbf::SafetyFilter filter(c);
    cbf::Vec7 nominal = cbf::Vec7::Zero();
    nominal[table ? 1 : 0] = .2;
    const auto r = filter.compute(m.view(), nominal, box, .001, 1);
    ASSERT_TRUE(r.accepted());
    EXPECT_TRUE(filter.verify_command(r.torque));
    EXPECT_GT(r.outside_domain_rows, 0);
    EXPECT_NEAR(r.torque[table ? 1 : 0], .2, 1e-6);
  }
}
TEST_P(BoundedRecovery, PersistentInwardCommandsDoNotLatchAFaultAndReversalWorks) {
  auto c = config();
  auto p = peer();
  cbf::SafetyFilter filter(c);
  double x = .085, v = 0;
  for (int cycle = 0; cycle < 4000; ++cycle) {
    auto m = plant(x, v);
    cbf::Vec7 nominal = cbf::Vec7::Zero();
    nominal[0] = cycle < 3000 ? -.2 : .2;
    const auto r = filter.compute(m.view(), nominal, box, .001, 1, &p);
    ASSERT_TRUE(r.accepted()) << cycle;
    ASSERT_TRUE(filter.verify_command(r.torque));
    x += v * .001 + .5 * r.torque[0] * 1e-6;
    v += r.torque[0] * .001;
    if (cycle < 3000)
      EXPECT_GE(x, .085 - 1e-8);
  }
  EXPECT_GT(x, .09);
}

TEST_P(BoundedRecovery, SelfCollisionAndSingularitySupportRecovery) {
  {
    auto c = config();
    c.inter_arm.enabled = false;
    c.self_collision.enabled = true;
    cbf::BodySphere base;
    base.frame = -1;
    base.radius = .04;
    base.group = 0;
    c.body_spheres[0].group = 1;
    c.body_spheres.insert(c.body_spheres.begin(), base);
    c.self_collision.clearance = .01;
    auto m = plant(.085);
    cbf::SafetyFilter filter(c);
    cbf::Vec7 nominal = cbf::Vec7::Zero();
    nominal[0] = .2;
    const auto r = filter.compute(m.view(), nominal, box, .001, 1);
    ASSERT_TRUE(r.accepted());
    EXPECT_TRUE(filter.verify_command(r.torque));
    EXPECT_GT(r.outside_domain_rows, 0);
    EXPECT_NEAR(r.torque[0], .2, 1e-6);
  }
  {
    auto c = config();
    c.inter_arm.enabled = false;
    c.body_spheres.clear();
    c.singularity.enabled = true;
    cbf_test::Model m;
    cbf_test::refresh(m);
    const auto k = cbf::singularity_kinematics(m.view(), c.singularity_length);
    ASSERT_TRUE(k.valid);
    c.singularity_minimum = std::sqrt(k.value) + .01;
    cbf::SafetyFilter filter(c);
    const auto r = filter.compute(m.view(), cbf::Vec7::Zero(), box, .001, 1);
    ASSERT_TRUE(r.accepted());
    EXPECT_TRUE(filter.verify_command(r.torque));
    EXPECT_GT(r.outside_domain_rows, 0);
  }
}
TEST_P(BoundedRecovery, AccelerationConstraintUsesBestAvailableBrakingWhenActuationIsLimited) {
  auto c = config();
  c.inter_arm.enabled = false;
  c.body_spheres.clear();
  c.acceleration.enabled = true;
  c.acceleration_limits.setConstant(.1);
  auto m = plant(.2);
  cbf::SafetyFilter filter(c);
  auto limited = box;
  limited.lower[0] = .15;
  limited.upper[0] = .85;
  const auto r = filter.compute(m.view(), cbf::Vec7::Constant(.5), limited, .001, 1);
  ASSERT_TRUE(r.accepted());
  EXPECT_TRUE(filter.verify_command(r.torque));
  EXPECT_NEAR(r.torque[0], .15, 1e-6);
  EXPECT_GT(r.recovery_rows, 0);
  EXPECT_GT(r.max_requested_violation, .049);
}

TEST(RecoveryProjection, RequestedTableRowIsNotWeakenedWhenAnotherFeasibleTorqueExists) {
  cbf::ConstraintSet rows;
  rows.count = 1;
  rows.rows[0].g = cbf::Vec7::Unit(0);
  rows.rows[0].family = cbf::Family::table;
  rows.requested_bounds[0] = 0;
  rows.braking_anchor = cbf::Vec7::Unit(0);
  rows.rows[0].bound = 1;  // Anchor allowance must not admit the raw inward request.
  const auto output =
      cbf::recovery_projection(rows, cbf::Vec7::Constant(.5), box, cbf::Vec7::Ones(), 6);
  EXPECT_NEAR(output[0], 0, 1e-12);
  EXPECT_NEAR(output[1], .5, 1e-12);  // Feasible tangent component is retained.
}

INSTANTIATE_TEST_SUITE_P(TorqueMethods, BoundedRecovery,
                         ::testing::Values(cbf::RecoveryTorqueMethod::projection,
                                           cbf::RecoveryTorqueMethod::native_qp));

TEST(NativeRecoveryQp, UsesNativeSolverAndReportsActualInfeasibilitySeparately) {
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  
  c.recovery_torque_method = cbf::RecoveryTorqueMethod::native_qp;
  c.analytic_nominal = false;  // Exercise the real backend rather than its shortcut.
  c.acceleration.enabled = true;
  c.acceleration_limits.setConstant(.1);
  c.max_filter_time_us = c.max_model_age_us = 1000000;
  auto m = plant(.2);
  cbf::SafetyFilter filter(c);
  const auto feasible = filter.compute(m.view(), cbf::Vec7::Constant(.5), box, .001, 1);
  ASSERT_TRUE(feasible.accepted()) << cbf::name(feasible.status);
  EXPECT_EQ(feasible.requested_qp_status, cbf::Status::solved_hard);
  EXPECT_EQ(feasible.recovery_qp_status, cbf::Status::disabled);
  EXPECT_FALSE(feasible.analytic_solution);
  EXPECT_FALSE(feasible.recovery_projection_fallback);
  EXPECT_GT(feasible.iterations, 0);
  EXPECT_NEAR(feasible.torque[0], .1, 1e-6);
  EXPECT_TRUE(filter.verify_command(feasible.torque));
  auto limited = box;
  limited.lower[0] = .15;
  const auto recovery = filter.compute(m.view(), cbf::Vec7::Constant(.5), limited, .001, 1);
  ASSERT_TRUE(recovery.accepted()) << cbf::name(recovery.status);
  EXPECT_EQ(recovery.requested_qp_status, cbf::Status::infeasible);
  EXPECT_EQ(recovery.recovery_qp_status, cbf::Status::solved_hard);
  EXPECT_FALSE(recovery.recovery_projection_fallback);
  EXPECT_NEAR(recovery.torque[0], .15, 1e-6);
  EXPECT_GT(recovery.max_requested_violation, .049);
  EXPECT_TRUE(filter.verify_command(recovery.torque));
}

TEST(NativeRecoveryQp, KeepsOldShortcutAndUsesFullQpForIntersectingConstraints) {
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  
  c.recovery_torque_method = cbf::RecoveryTorqueMethod::native_qp;
  c.acceleration.enabled = true;
  c.acceleration_limits.setConstant(.1);
  c.max_filter_time_us = c.max_model_age_us = 1000000;
  cbf::Vec7 nominal = cbf::Vec7::Zero();
  nominal[0] = nominal[1] = .5;
  auto m = plant(.2);
  m.mass(0, 0) = m.mass(1, 1) = 2;
  m.mass(0, 1) = m.mass(1, 0) = 1;
  cbf::SafetyFilter filter(c);
  const auto r = filter.compute(m.view(), nominal, box, .001, 1);
  ASSERT_TRUE(r.accepted()) << cbf::name(r.status);
  EXPECT_FALSE(r.analytic_solution);  // The single-row optimum violates the other row.
  EXPECT_GT(r.iterations, 0);
  EXPECT_FALSE(r.recovery_projection_fallback);
  EXPECT_NEAR(r.torque[0], .3, 1e-6);
  EXPECT_NEAR(r.torque[1], .3, 1e-6);
  EXPECT_TRUE(filter.verify_command(r.torque));
  nominal.setZero();
  nominal[0] = .5;
  auto independent = plant(.2);
  const auto shortcut = filter.compute(independent.view(), nominal, box, .001, 1);
  ASSERT_TRUE(shortcut.accepted());
  EXPECT_TRUE(shortcut.analytic_solution);
  EXPECT_EQ(shortcut.iterations, 0);
  EXPECT_NEAR(shortcut.torque[0], .1, 1e-6);
}

TEST(NativeRecoveryQp, IterationLimitUsesVerifiedBackupWithoutClaimingInfeasibility) {
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  
  c.recovery_torque_method = cbf::RecoveryTorqueMethod::native_qp;
  c.analytic_nominal = false;
  c.qpoases_recalculations = 1;
  c.acceleration.enabled = true;
  c.acceleration_limits.setConstant(.1);
  c.max_filter_time_us = c.max_model_age_us = 1000000;
  auto m = plant(.2);
  m.mass(0, 0) = m.mass(1, 1) = 2;
  m.mass(0, 1) = m.mass(1, 0) = 1;
  cbf::Vec7 nominal = cbf::Vec7::Zero();
  nominal[0] = nominal[1] = .5;
  cbf::SafetyFilter filter(c);
  const auto r = filter.compute(m.view(), nominal, box, .001, 1);
  ASSERT_TRUE(r.accepted()) << cbf::name(r.status);
  EXPECT_EQ(r.requested_qp_status, cbf::Status::iteration_limit);
  EXPECT_EQ(r.recovery_qp_status, cbf::Status::disabled);
  EXPECT_TRUE(r.recovery_projection_fallback);
  EXPECT_TRUE(filter.verify_command(r.torque));
}

TEST(CapsuleCover, ParallelSegmentsBlockSideApproachAndHeldOwnerOmitsCorrection) {
  cbf::Configuration c;
  c.uses_body_volumes = c.inter_arm.enabled = true;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  c.inter_arm.clearance = .01;
  cbf::BodySphere segment;
  segment.frame = 0;
  segment.span = Eigen::Vector3d(0, 0, .12);
  segment.radius = .04;
  c.body_spheres.push_back(segment);
  auto remote = peer();
  remote.spheres[0].position.z() = .025;
  remote.spheres[0].span.z() = .12;
  auto m = plant(.085);
  const cbf::Vec7 inward = -.2 * cbf::Vec7::Unit(0);
  cbf::SafetyFilter filter(c);
  auto solved = filter.compute(m.view(), inward, box, .001, 1, &remote);
  ASSERT_TRUE(solved.accepted());
  EXPECT_TRUE(filter.verify_command(solved.torque));
  EXPECT_NEAR(solved.torque[0], 0, 1e-9);
  EXPECT_GT(solved.outside_domain_rows, 0);
  solved = filter.compute(m.view(), -inward, box, .001, 1, &remote);
  ASSERT_TRUE(solved.accepted());
  EXPECT_NEAR(solved.torque[0], .2, 1e-9);
  ASSERT_EQ(filter.prepare(m.view(), box, .001, 1, &remote, nullptr, false),
            cbf::Status::solved_hard);
  EXPECT_NEAR(filter.solve_prepared(inward).torque[0], -.2, 1e-9);
}

TEST(CapsuleCover, FreshWidthMovesEachFingerAndStaleWidthCoversFullTravel) {
  cbf::Configuration c;
  cbf::BodySphere finger;
  finger.frame = 0;
  finger.radius = .014;
  finger.span.z() = .07;
  finger.opening_axis = Eigen::Vector3d::UnitY();
  auto m = plant(.2);
  auto view = m.view();
  view.timestamp_ns = 50000000;
  view.finger_opening = {.06, 49000000};
  auto fresh = cbf::position_body_volume(finger, view, c);
  EXPECT_NEAR(fresh.offset.y(), .03, 1e-12);
  EXPECT_LT(fresh.radius, .016);
  EXPECT_EQ(fresh.span, finger.span);
  view.finger_opening = {.06, 1};
  auto stale = cbf::position_body_volume(finger, view, c);
  EXPECT_NEAR(stale.offset.y(), .02, 1e-12);
  EXPECT_NEAR(stale.radius, finger.radius + .02, 1e-12);
}
