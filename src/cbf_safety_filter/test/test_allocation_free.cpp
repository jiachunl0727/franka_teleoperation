#include <dlfcn.h>
#include <gtest/gtest.h>

#include <cstdlib>

#include "cbf_safety_filter/safety_filter.hpp"
#include "fixture.hpp"
namespace cbf = cbf_safety_filter;
TEST(AllocationAudit, AllBackendsReportSteadyStateAllocations) {
  using Begin = void (*)();
  using End = uint64_t (*)();
  auto begin = reinterpret_cast<Begin>(dlsym(RTLD_DEFAULT, "cbf_allocation_begin"));
  auto end = reinterpret_cast<End>(dlsym(RTLD_DEFAULT, "cbf_allocation_end"));
  ASSERT_NE(begin, nullptr);
  ASSERT_NE(end, nullptr);
  for (auto backend : {cbf::Backend::osqp, cbf::Backend::qpoases, cbf::Backend::piqp}) {
    cbf::Configuration c;
    c.analytic_nominal = false;
    c.backend = backend;
    c.max_constraints = 28;
    c.max_filter_time_us = 1e8;
    c.position.enabled = c.velocity.enabled = true;
    c.osqp_iterations = 20000;
    c.piqp_iterations = 200;
    c.osqp_abs = c.osqp_rel = c.piqp_abs = c.piqp_rel = c.qpoases_tolerance = 1e-9;
    cbf::SafetyFilter filter(c);
    cbf_test::Model m;
    m.q[0] = 2.7963;
    cbf_test::refresh(m);
    cbf::TorqueBounds bounds{cbf::Vec7::Constant(-2), cbf::Vec7::Constant(2)};
    filter.compute(m.view(), cbf::Vec7::Zero(), bounds, .001, 1);
    uint64_t count = 0;
    bool accepted = true;
    for (int i = 0; i < 20; ++i) {
      const cbf::Vec7 nominal = cbf::Vec7::Constant(i % 2 ? .2 : -.2);
      begin();
      const auto& result = filter.compute(m.view(), nominal, bounds, .001, 1);
      count += end();
      accepted &= result.accepted();
    }
    EXPECT_TRUE(accepted) << cbf::name(backend);
    RecordProperty(std::string(cbf::name(backend)) + "_allocations_20_calls", count);
  }
}

TEST(AllocationAudit, CollisionAndSingularityStagesAllocateNothing) {
  using Begin = void (*)();
  using End = uint64_t (*)();
  auto begin = reinterpret_cast<Begin>(dlsym(RTLD_DEFAULT, "cbf_allocation_begin"));
  auto end = reinterpret_cast<End>(dlsym(RTLD_DEFAULT, "cbf_allocation_end"));
  ASSERT_NE(begin, nullptr);
  ASSERT_NE(end, nullptr);
  auto c = cbf::load_configuration(CBF_TEST_CONFIG);
  c.max_constraints = 256;
  c.self_collision.enabled = c.inter_arm.enabled = c.singularity.enabled = true;
  cbf_test::Model m;
  cbf_test::refresh(m);
  auto peer = cbf::collision_snapshot(m.view(), c);
  for (int i = 0; i < peer.count; ++i)
    peer.spheres[i].position.y() += 1.2;
  cbf::ConstraintBuilder b(c);
  cbf::TorqueBounds box{-cbf::Vec7::Constant(.315), cbf::Vec7::Constant(.315)};
  b.build(m.view(), .001, 1, &box, &peer);
  begin();
  bool success = true;
  for (int i = 0; i < 100; ++i) {
    const auto snapshot = cbf::collision_snapshot(m.view(), c);
    const auto& rows = b.build(m.view(), .001, 1, &box, &peer);
    success &= snapshot.valid && rows.status == cbf::Status::solved_hard;
    const auto projected = b.reference_normals.project(cbf::Vec6::Ones());
    success &= projected.allFinite();
  }
  const auto allocations = end();
  EXPECT_TRUE(success);
  EXPECT_EQ(allocations, 0u);
  RecordProperty("avoidance_assembly_allocations_100_calls", allocations);
}

TEST(AllocationAudit, NearTableReferenceAndRejectedRowCaptureAllocateNothing) {
  using Begin = void (*)();
  using End = uint64_t (*)();
  auto begin = reinterpret_cast<Begin>(dlsym(RTLD_DEFAULT, "cbf_allocation_begin"));
  auto end = reinterpret_cast<End>(dlsym(RTLD_DEFAULT, "cbf_allocation_end"));
  ASSERT_NE(begin, nullptr);
  ASSERT_NE(end, nullptr);
  cbf_test::Model m;
  cbf_test::refresh(m);
  cbf::Configuration config;
  config.table.enabled = true;
  config.capsules.push_back({8, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), .02});
  config.table.height = m.poses[8].translation().z() - .02 - config.table.clearance - .001;
  cbf::ConstraintBuilder builder(config);
  const cbf::Vec7 gradient = m.jacobians[8].row(2).transpose();
  const cbf::Vec7 downward = -.02 * gradient / gradient.squaredNorm();
  bool valid = true;
  begin();
  for (int i = 0; i < 100; ++i) {
    m.dq.setZero();
    valid &= builder.build(m.view(), .001, 1).status == cbf::Status::solved_hard;
    valid &= builder.reference_normals.count > 0;
    const auto projected = builder.reference_normals.project(-cbf::Vec6::Unit(2));
    valid &= projected.allFinite();
    m.dq = downward;
    valid &= builder.build(m.view(), .001, 1).status == cbf::Status::invalid_domain;
    const cbf::ConstraintFailure copied = builder.current().failure;
    valid &= copied.present && copied.row.h > 0 && copied.row.psi < 0;
  }
  const auto allocations = end();
  EXPECT_TRUE(valid);
  EXPECT_EQ(allocations, 0u);
  RecordProperty("table_reference_and_failure_allocations_100_calls", allocations);
}

TEST(AllocationAudit, SingleHardBoundaryProjectionAllocatesNothing) {
  using Begin = void (*)();
  using End = uint64_t (*)();
  auto begin = reinterpret_cast<Begin>(dlsym(RTLD_DEFAULT, "cbf_allocation_begin"));
  auto end = reinterpret_cast<End>(dlsym(RTLD_DEFAULT, "cbf_allocation_end"));
  ASSERT_NE(begin, nullptr);
  ASSERT_NE(end, nullptr);
  cbf::Configuration c;
  c.backend = cbf::Backend::qpoases;
  c.position.enabled = true;
  c.max_filter_time_us = c.max_model_age_us = 1e6;
  cbf::SafetyFilter filter(c);
  cbf_test::Model m;
  m.q[0] = 2.8973 - .1 - .01;
  cbf_test::refresh(m);
  const cbf::TorqueBounds box{-cbf::Vec7::Ones(), cbf::Vec7::Ones()};
  begin();
  bool accepted = true;
  for (int i = 0; i < 100; ++i) {
    const auto& result = filter.compute(m.view(), cbf::Vec7::Ones(), box, .001, 1);
    accepted &= result.accepted() && result.analytic_solution && result.iterations == 0;
    accepted &= filter.hold_correction_is_local(cbf::Vec7::Ones());
  }
  const auto allocations = end();
  EXPECT_TRUE(accepted);
  EXPECT_EQ(allocations, 0u);
}

TEST(AllocationAudit, SnapshotRecoveryComputeAllocatesNothingAfterConstruction) {
  using Begin = void (*)();
  using End = uint64_t (*)();
  auto begin = reinterpret_cast<Begin>(dlsym(RTLD_DEFAULT, "cbf_allocation_begin"));
  auto end = reinterpret_cast<End>(dlsym(RTLD_DEFAULT, "cbf_allocation_end"));
  ASSERT_NE(begin, nullptr);
  ASSERT_NE(end, nullptr);
  cbf::Configuration c;
  c.inter_arm.enabled = true;
  c.max_filter_time_us = c.max_model_age_us = 1000000;
  cbf::BodySphere body;
  body.frame = 8;
  body.radius = .04;
  c.body_spheres = {body};
  cbf_test::Model m;
  cbf_test::refresh(m);
  auto peer = cbf::collision_snapshot(m.view(), c);
  peer.spheres[0].position.x() += .08 + c.inter_arm.clearance - .002;
  const cbf::TorqueBounds box{cbf::Vec7::Constant(-1), cbf::Vec7::Constant(1)};
  cbf::SafetyFilter filter(c);
  filter.compute(m.view(), cbf::Vec7::Constant(.2), box, .001, 1, &peer);
  bool accepted = true;
  begin();
  for (int i = 0; i < 20; ++i) {
    const auto& r = filter.compute(m.view(), cbf::Vec7::Constant(.2), box, .001, 1, &peer);
    accepted &= r.accepted() && filter.verify_command(r.torque);
  }
  const auto allocations = end();
  EXPECT_TRUE(accepted);
  EXPECT_EQ(allocations, 0u);
}
