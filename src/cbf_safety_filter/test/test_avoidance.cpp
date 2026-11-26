#include <gtest/gtest.h>

#include <chrono>

#include "cbf_safety_filter/safety_filter.hpp"
#include "fixture.hpp"
using namespace cbf_test;
namespace {
cbf::Configuration config() {
  cbf::Configuration c;
  c.max_constraints = 128;
  c.max_filter_time_us = 1e6;
  c.max_model_age_us = 1e6;
  c.osqp_iterations = 10000;
  c.osqp_abs = c.osqp_rel = 1e-9;
  c.piqp_iterations = 100;
  c.piqp_abs = c.piqp_rel = 1e-9;
  c.qpoases_recalculations = 500;
  return c;
}
cbf::BodySphere sphere(int frame, Eigen::Vector3d offset, int group, double radius = .03) {
  cbf::BodySphere b;
  b.frame = frame;
  b.offset = offset;
  b.group = group;
  b.radius = radius;
  return b;
}
}  // namespace
TEST(Avoidance, GroupPruningMatchesIndividualRowsDomainsAndReferenceNormals) {
  int pruned = 0;
  for (int sample = 0; sample < 160; ++sample) {
    Model m;
    m.q[0] += .01 * sample;
    m.dq = cbf::Vec7::Constant(.002 * (sample % 9 - 4));
    m.mass.diagonal() << 2., 1.7, 1.3, 1., .8, .6, .4;
    refresh(m);
    auto c = config();
    c.inter_arm.enabled = true;
    c.max_constraints = 512;
    for (int frame = 0; frame < 9; ++frame)
      for (int p = 0; p < 3; ++p)
        c.body_spheres.push_back(sphere(frame, {.01 * p, -.01 * p, .02 * p}, frame, .015));
    auto pc = c;
    pc.world_from_base.translation().y() = .15 + .025 * sample;
    auto peer = cbf::collision_snapshot(m.view(), pc);
    if (sample % 5 == 0)
      for (int j = 0; j < peer.count; ++j)
        peer.spheres[j].group = 999;  // Untrusted grouping hint.
    cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
    cbf::ConstraintBuilder fast(c), reference(c, false);
    const auto& a = fast.build(m.view(), .001, 1, &box, &peer);
    const auto& b = reference.build(m.view(), .001, 1, &box, &peer);
    ASSERT_EQ(a.status, b.status) << sample;
    ASSERT_EQ(a.count, b.count) << sample;
    for (int i = 0; i < a.count; ++i) {
      EXPECT_EQ(a.rows[i].family, b.rows[i].family);
      EXPECT_EQ(a.rows[i].g, b.rows[i].g);
      EXPECT_EQ(a.rows[i].bound, b.rows[i].bound);
      EXPECT_EQ(a.rows[i].h, b.rows[i].h);
      EXPECT_EQ(a.rows[i].psi, b.rows[i].psi);
    }
    ASSERT_EQ(fast.reference_normals.count, reference.reference_normals.count);
    for (int i = 0; i < fast.reference_normals.count; ++i)
      EXPECT_EQ(fast.reference_normals.normals[i], reference.reference_normals.normals[i]);
    pruned += fast.group_pruned_pairs;
  }
  EXPECT_GT(pruned, 10000);
}

TEST(Avoidance, SharedGeometryExchangePublishesCurrentValuesAndRejectsStalePeer) {
  Model m;
  m.dq = cbf::Vec7::Constant(.001);
  refresh(m);
  auto c = config();
  c.inter_arm.enabled = true;
  c.body_spheres = {sphere(8, {.02, 0, .03}, 1), sphere(-1, {0, 0, 0}, 0)};
  auto pc = c;
  pc.world_from_base.translation().y() = 2;
  const uint64_t stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
  auto view = m.view();
  view.timestamp_ns = stamp;
  auto peer = cbf::collision_snapshot(view, pc);
  struct Context {
    cbf::PeerSnapshot own;
    const cbf::PeerSnapshot* peer;
    int calls{0};
    const cbf::PeerSnapshot* latest{nullptr};
  } ctx{{}, &peer};
  const cbf::PeerExchange exchange{&ctx, [](void* data, const cbf::PeerSnapshot& own) noexcept {
                                     auto& c = *static_cast<Context*>(data);
                                     c.own = own;
                                     ++c.calls;
                                     return c.peer;
                                   }};
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.35), cbf::Vec7::Constant(.35)};
  cbf::ConstraintBuilder builder(c);
  ASSERT_EQ(builder.build(view, .001, stamp, &box, nullptr, &exchange).status,
            cbf::Status::solved_hard);
  EXPECT_EQ(ctx.calls, 1);
  const auto expected = cbf::collision_snapshot(view, c);
  ASSERT_EQ(ctx.own.count, expected.count);
  EXPECT_EQ(ctx.own.timestamp_ns, stamp);
  for (int i = 0; i < expected.count; ++i) {
    EXPECT_LT((ctx.own.spheres[i].position - expected.spheres[i].position).norm(), 1e-12);
    EXPECT_LT((ctx.own.spheres[i].velocity - expected.spheres[i].velocity).norm(), 1e-12);
  }
  const auto current_peer = peer;
  ctx.latest = &current_peer;
  peer.timestamp_ns = stamp - 4000000;
  EXPECT_EQ(builder.build(view, .001, stamp, &box, nullptr, &exchange).status,
            cbf::Status::stale_peer);
  auto refreshed = exchange;
  refreshed.read_latest = [](void* data) noexcept { return static_cast<Context*>(data)->latest; };
  EXPECT_EQ(builder.build(view, .001, stamp, &box, nullptr, &refreshed).status,
            cbf::Status::solved_hard);
}

TEST(Avoidance, AnalyticSingularityDerivativesAndLowerBound) {
  for (int sample = 0; sample < 20; ++sample) {
    Model m;
    m.q[0] += .017 * sample;
    m.q[1] += .01 * sample;
    m.q[3] -= .012 * sample;
    m.dq << .12, -.13, .21, .08, -.05, .17, .11;
    refresh(m);
    auto k = cbf::singularity_kinematics(m.view(), .3);
    ASSERT_TRUE(k.valid);
    cbf::Jacobian j = m.jacobians[8];
    j.topRows<3>() /= .3;
    Eigen::JacobiSVD<cbf::Jacobian> svd(j);
    EXPECT_LE(k.value, svd.singularValues()[5] * svd.singularValues()[5] + 1e-12);
    for (int a = 0; a < 7; ++a) {
      Model plus = m, minus = m;
      plus.q[a] += 1e-5;
      minus.q[a] -= 1e-5;
      refresh(plus);
      refresh(minus);
      const double fd = (cbf::singularity_kinematics(plus.view(), .3).value -
                         cbf::singularity_kinematics(minus.view(), .3).value) /
                        2e-5;
      EXPECT_NEAR(k.gradient[a], fd, 1e-7);
    }
    Model plus = m, minus = m;
    const double dt = 1e-3;
    plus.q += dt * m.dq;
    minus.q -= dt * m.dq;
    refresh(plus);
    refresh(minus);
    const double fd = (cbf::singularity_kinematics(plus.view(), .3).value - 2 * k.value +
                       cbf::singularity_kinematics(minus.view(), .3).value) /
                      (dt * dt);
    EXPECT_NEAR(k.bias, fd, 1e-7);
  }
}
TEST(Avoidance, SingularAndUnsafeStartRejected) {
  Model m;
  m.q.setZero();
  refresh(m);
  auto c = config();
  c.singularity.enabled = true;
  cbf::ConstraintBuilder b(c);
  EXPECT_EQ(b.build(m.view(), .001, 1).status, cbf::Status::invalid_domain);
  refresh(m);
  c = config();
  c.self_collision.enabled = true;
  c.body_spheres = {sphere(8, Eigen::Vector3d::Zero(), 1), sphere(-1, m.poses[8].translation(), 5)};
  cbf::ConstraintBuilder collision(c);
  EXPECT_EQ(collision.build(m.view(), .001, 1).status, cbf::Status::invalid_domain);
}
TEST(Avoidance, SphereRelativeDerivativeMatchesIndependentFk) {
  Model m;
  m.dq << .01, -.02, .01, .03, .02, .01, -.02;
  refresh(m);
  auto c = config();
  c.self_collision.enabled = true;
  c.self_collision.clearance = .01;
  c.body_spheres = {sphere(8, Eigen::Vector3d::Zero(), 1),
                    sphere(-1, m.poses[8].translation() + Eigen::Vector3d(.2, 0, 0), 5)};
  cbf::ConstraintBuilder b(c);
  const auto set = b.build(m.view(), .001, 1);
  ASSERT_EQ(set.status, cbf::Status::solved_hard);
  ASSERT_EQ(set.count, 1);
  auto h = [&](const Model& x) {
    return (x.poses[8].translation() - c.body_spheres[1].offset).squaredNorm() - .07 * .07;
  };
  Model plus = m, minus = m;
  double dt = 1e-4;
  plus.q += dt * m.dq;
  minus.q -= dt * m.dq;
  refresh(plus);
  refresh(minus);
  const double hd = (h(plus) - h(minus)) / (2 * dt),
               hdd = (h(plus) - 2 * h(m) + h(minus)) / (dt * dt);
  EXPECT_NEAR(set.rows[0].bound, hdd + 10 * hd + 25 * h(m), 1e-6);
  for (int a = 0; a < 7; ++a) {
    plus = m;
    minus = m;
    plus.q[a] += dt;
    minus.q[a] -= dt;
    refresh(plus);
    refresh(minus);
    EXPECT_NEAR(-set.rows[0].g[a], (h(plus) - h(minus)) / (2 * dt), 1e-7);
  }
}
TEST(Avoidance, CollisionFilterIntervenesAllBackendsAndPreservesTangentialProjection) {
  Model m;
  refresh(m);
  for (auto backend : {cbf::Backend::osqp, cbf::Backend::qpoases, cbf::Backend::piqp}) {
    auto c = config();
    c.backend = backend;
    c.self_collision.enabled = true;
    c.self_collision.clearance = .01;
    c.body_spheres = {sphere(8, Eigen::Vector3d::Zero(), 1),
                      sphere(-1, m.poses[8].translation() + Eigen::Vector3d(.09, 0, 0), 5)};
    cbf::ConstraintBuilder b(c);
    auto set = b.build(m.view(), .001, 1);
    ASSERT_EQ(set.status, cbf::Status::solved_hard);
    cbf::Vec7 nominal = 10 * set.rows[0].g.normalized();
    cbf::SafetyFilter f(c);
    const auto& r =
        f.compute(m.view(), nominal, {-cbf::Vec7::Constant(20), cbf::Vec7::Constant(20)}, .001, 1);
    ASSERT_TRUE(r.accepted()) << cbf::name(r.status);
    EXPECT_GT(r.intervention_norm, .1);
    EXPECT_TRUE(f.verify_command(r.torque));
  }
  cbf::ReferenceNormals n;
  n.count = 1;
  n.normals[0] = cbf::Vec6::Unit(0);
  cbf::Vec6 request;
  request << -1, 2, 3, 0, 0, .2;
  const auto projected = n.project(request);
  EXPECT_NEAR(projected[0], 0, 1e-12);
  EXPECT_NEAR(projected[1], 2, 1e-12);
  EXPECT_NEAR(projected[5], .2, 1e-12);
  request[0] = 1;
  EXPECT_TRUE(n.project(request).isApprox(request));
}
TEST(Avoidance, PeerFreshnessAndMotionBoundsAreMandatory) {
  Model m;
  refresh(m);
  auto c = config();
  c.inter_arm.enabled = true;
  c.body_spheres = {sphere(8, Eigen::Vector3d::Zero(), 1)};
  auto peer = cbf::collision_snapshot(m.view(), c);
  peer.spheres[0].position.x() += 2;
  cbf::TorqueBounds box{-cbf::Vec7::Ones(), cbf::Vec7::Ones()};
  cbf::ConstraintBuilder b(c);
  EXPECT_EQ(b.build(m.view(), .001, 1, &box).status, cbf::Status::invalid_model);
  EXPECT_EQ(b.build(m.view(), .001, 1, &box, &peer).status, cbf::Status::solved_hard);
  peer.timestamp_ns = 2;
  EXPECT_EQ(b.build(m.view(), .001, 1, &box, &peer).status, cbf::Status::invalid_model);
  EXPECT_EQ(b.build(m.view(), .001, 2, &box, &peer).status, cbf::Status::solved_hard);
  peer.timestamp_ns = 0;
  EXPECT_EQ(b.build(m.view(), .001, 1, &box, &peer).status, cbf::Status::invalid_model);
  peer.timestamp_ns = 1;
  EXPECT_EQ(b.build(m.view(), .001, 4000001, &box, &peer).status, cbf::Status::stale_peer);
  peer.spheres[0].velocity.x() = 2;
  EXPECT_EQ(b.build(m.view(), .001, 1, &box, &peer).status, cbf::Status::invalid_model);
  peer.spheres[0].velocity.setZero();
  peer.spheres[0].position = m.poses[8].translation();
  EXPECT_EQ(b.build(m.view(), .001, 1, &box, &peer).status, cbf::Status::invalid_domain);
}
TEST(Avoidance, StrongerPeerAccelerationTightensRowsAndRedundancyIsExact) {
  Model m;
  refresh(m);
  auto c = config();
  c.inter_arm.enabled = true;
  c.max_constraints = 128;
  c.body_spheres = {sphere(8, Eigen::Vector3d::Zero(), 1)};
  auto peer = cbf::collision_snapshot(m.view(), c);
  peer.spheres[0].position.x() += 1;
  cbf::TorqueBounds box{-cbf::Vec7::Constant(20), cbf::Vec7::Constant(20)};
  cbf::ConstraintBuilder b(c);
  auto first = b.build(m.view(), .001, 1, &box, &peer);
  ASSERT_EQ(first.status, cbf::Status::solved_hard);
  peer.acceleration_limit += 1;
  auto second = b.build(m.view(), .001, 1, &box, &peer);
  ASSERT_EQ(second.status, cbf::Status::solved_hard);
  ASSERT_EQ(first.count, second.count);
  ASSERT_GT(second.count, 0);
  EXPECT_LT(second.rows[second.count - 1].bound, first.rows[first.count - 1].bound);
}
TEST(Avoidance, ShippedGeometryHomeAndSingleUseTransaction) {
  Model m;
  refresh(m);
  auto c = cbf::load_configuration(CBF_TEST_CONFIG);
  c.self_collision.enabled = true;
  c.singularity.enabled = true;
  c.max_filter_time_us = 1e6;
  c.osqp_iterations = 10000;
  ASSERT_EQ(c.body_spheres.size(), 15u);
  bool base = false, hand = false;
  for (auto b : c.body_spheres) {
    base |= b.frame == -1;
    hand |= b.group == 9;
  }
  EXPECT_TRUE(base);
  EXPECT_TRUE(hand);
  c.validate();
  cbf::SafetyFilter f(c);
  cbf::TorqueBounds box{-cbf::Vec7::Ones(), cbf::Vec7::Ones()};
  const auto& r = f.compute(m.view(), cbf::Vec7::Zero(), box, .001, 1);
  EXPECT_TRUE(r.accepted()) << cbf::name(r.status);
  EXPECT_FALSE(f.solve_prepared(cbf::Vec7::Zero()).accepted());
}
TEST(Avoidance, CollisionSlackAndInvalidSettingsRejected) {
  auto c = config();
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = config();
  c.point_acceleration_limit = 0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = config();
  c.self_collision.enabled = true;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(Avoidance, PreparedRowsCannotAuthorizeAnUnsolvedCommand) {
  Model m;
  refresh(m);
  auto c = config();
  cbf::SafetyFilter f(c);
  cbf::TorqueBounds box{-cbf::Vec7::Ones(), cbf::Vec7::Ones()};
  ASSERT_TRUE(f.compute(m.view(), cbf::Vec7::Zero(), box, .001, 1).accepted());
  ASSERT_TRUE(f.verify_command(cbf::Vec7::Zero()));
  ASSERT_EQ(f.prepare(m.view(), box, .001, 1), cbf::Status::solved_hard);
  EXPECT_FALSE(f.verify_command(cbf::Vec7::Zero()));
  EXPECT_TRUE(f.solve_prepared(cbf::Vec7::Zero()).accepted());
  EXPECT_TRUE(f.verify_command(cbf::Vec7::Zero()));
}

TEST(Avoidance, TableOnlyBlocksInwardReferenceEvenWhenTorqueRowsAreRedundant) {
  Model m;
  refresh(m);
  auto c = config();
  c.table.enabled = true;
  c.capsules.push_back({8, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), .02});
  c.table.height = m.poses[8].translation().z() - .02 - c.table.clearance - .02;
  cbf::TorqueBounds box{cbf::Vec7::Zero(), cbf::Vec7::Zero()};
  cbf::SafetyFilter filter(c);
  ASSERT_EQ(filter.prepare(m.view(), box, .001, 1), cbf::Status::solved_hard);
  const auto& normals = filter.reference_normals();
  ASSERT_GT(normals.count, 0);
  cbf::Vec6 inward = -cbf::Vec6::Unit(2);
  EXPECT_LT(normals.project(inward).norm(), 1e-6);
  EXPECT_TRUE(normals.project(-inward).isApprox(-inward, 1e-6));
  EXPECT_TRUE(normals.project(cbf::Vec6::Unit(0)).isApprox(cbf::Vec6::Unit(0), 1e-6));
  ASSERT_TRUE(filter.solve_prepared(cbf::Vec7::Zero()).accepted());
  EXPECT_TRUE(filter.verify_command(cbf::Vec7::Zero()));
}

TEST(Avoidance, SingularityReferenceAllowsOnlyNonworseningMotionNearThreshold) {
  Model m;
  refresh(m);
  auto c = config();
  c.singularity.enabled = true;
  const auto metric = cbf::singularity_kinematics(m.view(), c.singularity_length);
  ASSERT_TRUE(metric.valid);
  c.singularity_minimum = .9 * std::sqrt(metric.value);  // Explicit virtual threshold.
  cbf::SafetyFilter filter(c);
  cbf::TorqueBounds box{-cbf::Vec7::Ones(), cbf::Vec7::Ones()};
  ASSERT_EQ(filter.prepare(m.view(), box, .001, 1), cbf::Status::solved_hard);
  const auto& normals = filter.reference_normals();
  ASSERT_GT(normals.count, 0);
  const cbf::Vec6 n = normals.normals[0];
  EXPECT_LT(normals.project(-n).norm(), 1e-10);
  EXPECT_TRUE(normals.project(n).isApprox(n, 1e-10));
  cbf::Vec6 tangent = cbf::Vec6::Ones();
  tangent -= n * n.dot(tangent);
  EXPECT_TRUE(normals.project(tangent).isApprox(tangent, 1e-10));
}

TEST(Avoidance, SingularityTorqueInequalityMatchesDirectionalSecondDerivative) {
  Model m;
  m.dq << .01, -.02, .01, .03, .01, -.02, .01;
  refresh(m);
  auto c = config();
  c.singularity.enabled = true;
  const auto metric = cbf::singularity_kinematics(m.view(), c.singularity_length);
  cbf::ConstraintBuilder b(c);
  auto rows = b.build(m.view(), .001, 1);
  ASSERT_EQ(rows.status, cbf::Status::solved_hard);
  ASSERT_EQ(rows.count, 1);
  EXPECT_TRUE(rows.rows[0].g.isApprox(-metric.gradient, 1e-10));
  EXPECT_NEAR(rows.rows[0].bound,
              metric.bias + 10 * metric.gradient.dot(m.dq) +
                  25 * (metric.value - c.singularity_minimum * c.singularity_minimum),
              1e-10);
  cbf::Vec7 tau = cbf::Vec7::Constant(.1);
  const double actual =
      metric.gradient.dot(tau) + metric.bias + 10 * metric.gradient.dot(m.dq) + 25 * rows.rows[0].h;
  EXPECT_NEAR(rows.rows[0].bound - rows.rows[0].g.dot(tau), actual, 1e-10);
}

TEST(Avoidance, RobustInterArmRowBoundsAdversarialPeerMotion) {
  Model m;
  refresh(m);
  auto c = config();
  c.inter_arm.enabled = true;
  c.body_spheres = {sphere(8, Eigen::Vector3d::Zero(), 1)};
  auto peer = cbf::collision_snapshot(m.view(), c);
  peer.spheres[0].position.x() += 1;
  cbf::TorqueBounds box{-cbf::Vec7::Constant(100), cbf::Vec7::Constant(100)};
  cbf::ConstraintBuilder builder(c);
  const auto set = builder.build(m.view(), .001, 1, &box, &peer);
  ASSERT_EQ(set.status, cbf::Status::solved_hard);
  const cbf::Row* row = nullptr;
  for (int i = 0; i < set.count; ++i)
    if (set.rows[i].family == cbf::Family::inter_arm)
      row = &set.rows[i];
  ASSERT_NE(row, nullptr);
  const auto p = cbf::point_kinematics(m.view(), c.body_spheres[0], c.world_from_base);
  for (int sample = 0; sample < 1000; ++sample) {
    Eigen::Vector3d a(std::sin(sample + .3), std::cos(2 * sample + .2), std::sin(3 * sample + .1));
    a.normalize();
    Eigen::Vector3d b(std::cos(3 * sample + .4), std::sin(sample + .7), std::cos(2 * sample + .8));
    b.normalize();
    const Eigen::Vector3d dp = a * peer.speed_limit * .001, dv = b * peer.acceleration_limit * .001;
    const Eigen::Vector3d peer_acc = a * peer.acceleration_limit;
    cbf::Vec7 tau;
    for (int j = 0; j < 7; ++j)
      tau[j] = 100 * std::sin(sample * .13 + j);
    const Eigen::Vector3d d = p.position - peer.spheres[0].position - dp;
    const Eigen::Vector3d v = p.velocity - peer.spheres[0].velocity - dv;
    const Eigen::Vector3d acceleration = p.jacobian * tau + p.acceleration_bias - peer_acc;
    const double radius = .06 + c.inter_arm.clearance;
    const double actual = 2 * v.squaredNorm() + 2 * d.dot(acceleration) + 10 * 2 * d.dot(v) +
                          25 * (d.squaredNorm() - radius * radius);
    const double robust = row->bound - row->g.dot(tau);
    EXPECT_GE(actual + 1e-9, robust);
  }
}
