#include <gtest/gtest.h>

#include <fstream>
#include <limits>

#include "cbf_safety_filter/safety_filter.hpp"
#include "teleop_controllers/inter_arm_policy.hpp"
#include "teleop_controllers/panda_limits.hpp"
#include "teleop_controllers/panda_model_fixture.hpp"
#include "teleop_controllers/urdf_model.hpp"
namespace ft = teleop_controllers;
namespace cbf = cbf_safety_filter;
TEST(InterArmPolicy, SeedsKeepRealTimestampAndRejectMovingStartup) {
  auto m = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  cbf::Configuration c;
  cbf::BodySphere body;
  body.frame = 8;
  body.radius = .03;
  c.body_spheres.push_back(body);
  auto view = [&]() {
    return cbf::ModelView{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, 1000000};
  };
  auto seed = ft::stationary_collision_seed(view(), c);
  ASSERT_TRUE(seed.valid);
  EXPECT_EQ(seed.timestamp_ns, 1000000u);
  m.dq[0] = .03;
  EXPECT_FALSE(ft::stationary_collision_seed(view(), c).valid);
  m.dq.setZero();
  seed = ft::stationary_collision_seed(view(), c);
  seed.spheres[0].position.x() += 1;
  c.inter_arm.enabled = true;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  cbf::ConstraintBuilder builder(c);
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  EXPECT_EQ(builder.build(view(), .001, 1000000, &box, &seed).status, cbf::Status::solved_hard);
  EXPECT_EQ(builder.build(view(), .001, 5000000, &box, &seed).status, cbf::Status::stale_peer);
}
TEST(InterArmPolicy, HeldLawIsNotReplacedByAvoidanceYield) {
  cbf::Vec7 hold = cbf::Vec7::Zero(), candidate = hold;
  EXPECT_TRUE(ft::preserves_held_arm(hold, candidate, 1e-6));
  candidate[3] = .02;
  EXPECT_FALSE(ft::preserves_held_arm(hold, candidate, 1e-6));
  candidate[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ft::preserves_held_arm(hold, candidate, 1e-6));
}
TEST(InterArmPolicy, MovingReferenceAvoidsFreshHeldPeerAndRetainsRetreat) {
  auto m = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  cbf::Configuration c;
  c.inter_arm.enabled = true;
  c.inter_arm_boundary_activation = .15;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  c.point_acceleration_limit = .1;
  cbf::BodySphere b;
  b.frame = 8;
  b.radius = .03;
  c.body_spheres.push_back(b);
  cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, 1000000};
  auto peer = cbf::collision_snapshot(view, c);
  peer.spheres[0].position.x() += .24;
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  cbf::SafetyFilter filter(c);
  ASSERT_EQ(filter.prepare(view, box, .001, 1000000, &peer), cbf::Status::solved_hard);
  const auto& normals = filter.reference_normals();
  ASSERT_GT(normals.count, 0);
  cbf::Vec6 toward = cbf::Vec6::Zero();
  toward[0] = .001;
  EXPECT_GE(normals.normals[0].dot(normals.project(toward)), -1e-10);
  EXPECT_GT(normals.project(-toward).norm(), .0009);
  const auto& result = filter.solve_prepared(m.coriolis);
  EXPECT_TRUE(result.accepted());
  EXPECT_TRUE(filter.verify_command(result.torque));
}

TEST(InterArmPolicy, FreshAcquisitionAfterBarrierRemovesOnlyBarrierAge) {
  auto own = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  auto changed_q = own.q;
  changed_q[0] += .1;
  auto observed_after_wait = ft::panda_model_fixture(changed_q);
  cbf::Configuration c;
  c.inter_arm.enabled = true;
  cbf::BodySphere b;
  b.frame = 8;
  b.radius = .03;
  c.body_spheres.push_back(b);
  auto peer_config = c;
  peer_config.world_from_base.translation().y() = 1.;
  const cbf::ModelView old_view{
      own.q, own.dq, own.mass, own.coriolis, own.frame_pose, own.frame_jacobian, 1000000};
  const auto old_seed = ft::stationary_collision_seed(old_view, peer_config);
  unsigned reads = 0;
  const auto refreshed = ft::acquire_stationary_collision_seed(
      [&]() {
        ++reads;
        return observed_after_wait;
      },
      []() { return uint64_t{4000000}; }, [](const ft::RobotModelState& state) { return state; },
      peer_config);
  ASSERT_EQ(reads, 1u);
  ASSERT_TRUE(refreshed.valid);
  EXPECT_EQ(refreshed.timestamp_ns, 4000000u);
  EXPECT_GT((refreshed.spheres[0].position - old_seed.spheres[0].position).norm(), 1e-4);
  const cbf::ModelView first_callback{
      own.q, own.dq, own.mass, own.coriolis, own.frame_pose, own.frame_jacobian, 6000000};
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  cbf::ConstraintBuilder builder(c);
  EXPECT_EQ(builder.build(first_callback, .001, 6000000, &box, &old_seed).status,
            cbf::Status::stale_peer);
  EXPECT_EQ(builder.build(first_callback, .001, 6000000, &box, &refreshed).status,
            cbf::Status::solved_hard);
  const cbf::ModelView late_callback{
      own.q, own.dq, own.mass, own.coriolis, own.frame_pose, own.frame_jacobian, 8000001};
  EXPECT_EQ(builder.build(late_callback, .001, 8000001, &box, &refreshed).status,
            cbf::Status::stale_peer);
}

TEST(InterArmPolicy, RefreshDoesNotHideMovingFeedbackOrReadFailure) {
  auto moving = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  moving.dq[0] = .03;
  cbf::Configuration c;
  cbf::BodySphere b;
  b.frame = 8;
  b.radius = .03;
  c.body_spheres.push_back(b);
  EXPECT_FALSE(ft::acquire_stationary_collision_seed(
                   [&]() { return moving; }, []() { return uint64_t{1}; },
                   [](const ft::RobotModelState& state) { return state; }, c)
                   .valid);
  EXPECT_THROW(
      ft::acquire_stationary_collision_seed(
          []() -> ft::RobotModelState { throw std::runtime_error("read failed"); },
          []() { return uint64_t{1}; }, [](const ft::RobotModelState& state) { return state; }, c),
      std::runtime_error);
}

TEST(InterArmPolicy, HeldArmAllowsIndependentLocalMotionLimiting) {
  auto m = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  c.inter_arm.enabled = true;
  c.point_acceleration_limit = .1;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  cbf::BodySphere body;
  body.frame = 8;
  body.radius = .03;
  c.body_spheres.push_back(body);
  cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, 1000000};
  auto peer = cbf::collision_snapshot(view, c);
  peer.spheres[0].position.x() += 3.;
  const cbf::Vec7 nominal = .22 * cbf::Vec7::Unit(1);
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  cbf::SafetyFilter filter(c);
  const auto& r = filter.compute(view, nominal, box, .001, 1000000, &peer);
  ASSERT_TRUE(r.accepted()) << cbf::name(r.status);
  ASSERT_TRUE(filter.verify_command(r.torque));
  ASSERT_FALSE(ft::preserves_held_arm(nominal, r.torque, c.hard_tolerance));
  // This is a local point-acceleration limit, not yielding to the peer.
  EXPECT_TRUE(filter.hold_correction_is_local(nominal));
  EXPECT_FALSE(filter.hold_correction_is_local(nominal + cbf::Vec7::Ones()));
  filter.prepare(view, box, .001, 1000000, &peer);
  EXPECT_FALSE(filter.hold_correction_is_local(nominal));
}

TEST(InterArmPolicy, HeldArmStillRefusesInterArmDrivenCorrection) {
  auto m = ft::panda_model_fixture(Eigen::Map<const ft::Vector7d>(ft::kHome.data()));
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  c.inter_arm.enabled = true;
  c.inter_arm.alpha1 = c.inter_arm.alpha2 = .8;
  c.point_acceleration_limit = .1;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  cbf::BodySphere body;
  body.frame = 8;
  body.radius = .03;
  c.body_spheres.push_back(body);
  cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, 1000000};
  auto peer = cbf::collision_snapshot(view, c);
  peer.spheres[0].position.x() += .3;
  const cbf::Vec7 nominal = cbf::Vec7::Zero();
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  cbf::SafetyFilter filter(c);
  const auto& r = filter.compute(view, nominal, box, .001, 1000000, &peer);
  ASSERT_TRUE(r.accepted());
  ASSERT_TRUE(filter.verify_command(r.torque));
  ASSERT_FALSE(ft::preserves_held_arm(nominal, r.torque, c.hard_tolerance));
  EXPECT_FALSE(filter.hold_correction_is_local(nominal));
}

TEST(InterArmPolicy, AnalyticLocalOptimumRemainsLocalWithInterArmRowsPresent) {
  // Linear point model isolates the optimizer/policy seam from robot FK.
  ft::RobotModelState m;
  m.frame_pose[8].translation().x() = .4;
  m.frame_jacobian[8](0, 0) = 1.;
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  c.weights << 1.3, 2, 3, 4, 5, 6, 7;
  c.inter_arm.enabled = true;
  c.inter_arm.alpha1 = c.inter_arm.alpha2 = 2.;
  c.point_acceleration_limit = .1;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  cbf::BodySphere body;
  body.frame = 8;
  body.radius = .03;
  c.body_spheres.push_back(body);
  cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, 1000000};
  auto peer = cbf::collision_snapshot(view, c);
  peer.spheres[0].position.x() += .2;
  const cbf::Vec7 nominal = .2 * cbf::Vec7::Unit(0);
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  cbf::SafetyFilter filter(c);
  const auto& r = filter.compute(view, nominal, box, .001, 1000000, &peer);
  ASSERT_TRUE(r.accepted()) << cbf::name(r.status);
  ASSERT_TRUE(r.analytic_solution);
  bool inter_arm = false;
  for (int i = 0; i < r.rows; ++i)
    inter_arm |= r.families[i] == cbf::Family::inter_arm;
  ASSERT_TRUE(inter_arm);
  ASSERT_TRUE(filter.verify_command(r.torque));
  EXPECT_TRUE(filter.hold_correction_is_local(nominal));
}

TEST(InterArmPolicy, RecordedNativeReleaseBrakingAllowsVerifiedBoxRounding) {
  auto config = cbf::load_configuration(std::string(TELEOP_RELEASE_FIXTURE) + "/cbf.yaml");
  config.max_filter_time_us = config.max_model_age_us = 1e6;
  std::ifstream input(std::string(TELEOP_RELEASE_FIXTURE) + "/state.txt");
  ASSERT_TRUE(input.good());
  const auto read = [&](auto& values) {
    for (int i = 0; i < values.size(); ++i)
      input >> values[i];
  };
  cbf::Vec7 q, dq, nominal, coriolis;
  cbf::TorqueBounds bounds;
  read(q);
  read(dq);
  read(nominal);
  read(bounds.lower);
  read(bounds.upper);
  read(coriolis);
  cbf::Mat7 mass;
  for (int j = 0; j < 7; ++j)
    for (int k = 0; k < 7; ++k)
      input >> mass(j, k);
  cbf::PeerSnapshot peer;
  uint64_t age_ns;
  input >> peer.count >> peer.speed_limit >> peer.acceleration_limit >> age_ns;
  ASSERT_GT(peer.count, 0);
  ASSERT_LE(peer.count, cbf::kMaxPoints);
  for (int i = 0; i < peer.count; ++i) {
    read(peer.spheres[i].position);
    read(peer.spheres[i].velocity);
    input >> peer.spheres[i].radius;
  }
  ASSERT_FALSE(input.fail());
  constexpr uint64_t now = 1000000000;
  peer.valid = true;
  peer.timestamp_ns = now - age_ns;
  ft::UrdfPandaModel model(TELEOP_RELEASE_MODEL);
  ft::UrdfModelOutput state;
  ASSERT_TRUE(model.evaluate(q, dq, model.standard_flange_to_ee(), state));
  auto& m = state.state;
  m.mass = mass;
  m.coriolis = coriolis;
  const cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, now};
  cbf::SafetyFilter filter(config);
  const auto& result = filter.compute(view, nominal, bounds, .001, now, &peer);
  ASSERT_TRUE(result.accepted()) << cbf::name(result.status);
  ASSERT_FALSE(result.analytic_solution);
  for (int i = 0; i < result.rows; ++i)
    ASSERT_NE(result.families[i], cbf::Family::inter_arm);
  const cbf::Vec7 command = result.torque.cwiseMax(bounds.lower).cwiseMin(bounds.upper);
  const cbf::Vec7 clipped_nominal = nominal.cwiseMax(bounds.lower).cwiseMin(bounds.upper);
  ASSERT_LE((command - result.torque).cwiseAbs().maxCoeff(), config.hard_tolerance);
  ASSERT_TRUE(filter.verify_command(command));
  ASSERT_FALSE(ft::preserves_held_arm(clipped_nominal, command, config.hard_tolerance));
  // The native solver's recorded 8e-14 Nm bound rounding is already handled
  // by outgoing clamping/verification; it must not masquerade as peer yielding.
  EXPECT_TRUE(filter.hold_correction_is_local(nominal));
}

TEST(InterArmPolicy, SnapshotDataValidityDoesNotRejectAConstraintViolation) {
  cbf_safety_filter::PeerSnapshot snapshot;
  snapshot.valid = true;
  snapshot.timestamp_ns = 1;
  snapshot.count = 1;
  snapshot.speed_limit = 1;
  snapshot.acceleration_limit = 5;
  snapshot.spheres[0].radius = .04;
  snapshot.spheres[0].velocity.x() = 2;
  EXPECT_TRUE(teleop_controllers::valid_collision_snapshot(snapshot));
  snapshot.spheres[0].radius = -1;
  EXPECT_FALSE(teleop_controllers::valid_collision_snapshot(snapshot));
  snapshot.spheres[0].radius = .04;
  snapshot.valid = false;
  EXPECT_FALSE(teleop_controllers::valid_collision_snapshot(snapshot));
}
