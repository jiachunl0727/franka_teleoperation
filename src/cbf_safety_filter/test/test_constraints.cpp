#include <gtest/gtest.h>

#include "cbf_safety_filter/constraints.hpp"
#include "fixture.hpp"
namespace cbf = cbf_safety_filter;
TEST(Constraints, RecordedFingerClearanceUsesPhysicalTablePlaneWithoutAddedFingerAllowance) {
  cbf::Configuration c;
  c.table.enabled = true;
  c.table.height = .127;
  c.table.clearance = .005;
  c.capsules.push_back({7, {-.035355, -.035355, .1}, {.035355, .035355, .1}, .02});
  cbf_test::Model m;
  // Hardware shadow run 1, cycle 110645: minimum reported table margin.
  m.q << .13584938095536922, -.023596219283251594, .12617972544030914, -2.4688250157936085,
      -.10709367471594058, 2.5069529350149957, .19704698714071678;
  m.dq << -.006955642570298553, .0053210803609743335, -.0014971009473335635, -.000883647081617269,
      .00038868814455248277, .01467993082119124, .05930341404485522;
  cbf_test::refresh(m);
  cbf::ConstraintBuilder physical(c);
  const auto rows = physical.build(m.view(), .001, 1);
  ASSERT_EQ(rows.status, cbf::Status::solved_hard);
  ASSERT_EQ(rows.count, 2);
  EXPECT_NEAR(rows.rows[0].h, .05152765973075748, 1e-8);
  // The historical virtual-plane allowance duplicates the modeled fingers.
  c.table.height += .063;
  cbf::ConstraintBuilder duplicated(c);
  EXPECT_EQ(duplicated.build(m.view(), .001, 1).status, cbf::Status::invalid_domain);
}
TEST(Constraints, JointPositionVelocityAndAccelerationSignsAndDomains) {
  cbf::Configuration c;
  c.position.enabled = c.velocity.enabled = c.acceleration.enabled = true;
  cbf::ConstraintBuilder b(c);
  cbf_test::Model m;
  cbf_test::refresh(m);
  const auto rows = b.build(m.view(), .001, 1);
  ASSERT_EQ(rows.status, cbf::Status::solved_hard);
  ASSERT_EQ(rows.count, 42);
  for (int j = 0; j < 7; ++j) {
    EXPECT_NEAR(rows.rows[j * 6].g[j], 1, 1e-12);
    EXPECT_NEAR(rows.rows[j * 6 + 1].g[j], -1, 1e-12);
    const cbf::Vec7 acceleration = cbf::Vec7::Constant(.2);
    for (int r = 0; r < rows.count; ++r)
      EXPECT_LE(rows.rows[r].g.dot(acceleration), rows.rows[r].bound);
  }
  m.q[0] = 2.85;
  EXPECT_EQ(b.build(m.view(), .001, 1).status, cbf::Status::invalid_domain);
  m.q[0] = 2.6;
  m.dq[0] = 2.;
  EXPECT_EQ(b.build(m.view(), .001, 1).status, cbf::Status::invalid_domain);
  m.dq.setZero();
  m.mass(0, 0) = -1;
  EXPECT_EQ(b.build(m.view(), .001, 1).status, cbf::Status::invalid_model);
  m.mass.setIdentity();
  EXPECT_EQ(b.build(m.view(), .001, 3000000).status, cbf::Status::invalid_model);
}
TEST(Constraints, TableAndSquaredSphereAccelerationMatchFiniteDifferences) {
  cbf::Configuration c;
  c.table.enabled = c.obstacles.enabled = true;
  c.table.height = -1;
  c.capsules.push_back({7, {0, 0, 0}, {.02, .03, .1}, .03});
  c.points.push_back({8, {.01, .02, .03}, .02});
  c.obstacle_spheres.push_back({{2, 2, 2}, .1});
  c.pairs.push_back({0, 0});
  cbf::ConstraintBuilder b(c);
  cbf_test::Model m;
  m.dq << .1, -.1, .05, .1, -.04, .1, .02;
  cbf_test::refresh(m);
  const auto set = b.build(m.view(), .001, 1);
  ASSERT_EQ(set.status, cbf::Status::solved_hard);
  ASSERT_EQ(set.count, 3);
  const cbf::Vec7 qdd = cbf::Vec7::Constant(.1);
  const double eps = 1e-4;
  const auto plus = cbf_test::forward(m.q + eps * m.dq + .5 * eps * eps * qdd);
  const auto minus = cbf_test::forward(m.q - eps * m.dq + .5 * eps * eps * qdd);
  for (int row = 0; row < 3; ++row) {
    const int f = row < 2 ? 7 : 8;
    const Eigen::Vector3d offset = row == 0   ? c.capsules[0].a
                                   : row == 1 ? c.capsules[0].b
                                              : c.points[0].offset;
    const auto barrier = [&](const Eigen::Affine3d& pose) {
      const Eigen::Vector3d p = pose * offset;
      return row < 2 ? p.z() - c.table.height - .03 - c.table.clearance
                     : (p - c.obstacle_spheres[0].center).squaredNorm() -
                           std::pow(.02 + .1 + c.obstacles.clearance, 2);
    };
    const double h = barrier(m.poses[f]), hdot = (barrier(plus[f]) - barrier(minus[f])) / (2 * eps);
    const double hdd = (barrier(plus[f]) - 2 * h + barrier(minus[f])) / (eps * eps);
    const auto& family = row < 2 ? c.table : c.obstacles;
    const double expected =
        hdd + (family.alpha1 + family.alpha2) * hdot + family.alpha1 * family.alpha2 * h;
    EXPECT_NEAR(set.rows[row].bound - set.rows[row].g.dot(qdd), expected, 1e-5);
  }
}

TEST(Constraints, RetainsRejectedTableSpeedDomainAndClearsItOnNextPrepare) {
  cbf_test::Model m;
  cbf_test::refresh(m);
  cbf::Configuration c;
  c.table.enabled = true;
  c.capsules.push_back({8, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), .02});
  c.table.height = m.poses[8].translation().z() - .02 - c.table.clearance - .001;
  const cbf::Vec7 height_gradient = m.jacobians[8].row(2).transpose();
  m.dq = -.02 * height_gradient / height_gradient.squaredNorm();
  cbf::ConstraintBuilder builder(c);
  const auto& rejected = builder.build(m.view(), .001, 1);
  ASSERT_EQ(rejected.status, cbf::Status::invalid_domain);
  ASSERT_TRUE(rejected.failure.present);
  EXPECT_EQ(rejected.failure.row_index, 0);
  EXPECT_EQ(rejected.failure.row.family, cbf::Family::table);
  EXPECT_NEAR(rejected.failure.row.h, .001, 1e-12);
  EXPECT_LT(rejected.failure.row.psi, 0);
  EXPECT_EQ(rejected.count, 0);  // Failed row cannot become command authority.
  m.dq.setZero();
  EXPECT_EQ(builder.build(m.view(), .001, 1).status, cbf::Status::solved_hard);
  EXPECT_FALSE(builder.current().failure.present);
}

TEST(Constraints, EarlyModelFailureRetainsOnlyTheMeasuredPhase) {
  cbf::Configuration c;
  cbf::ConstraintBuilder builder(c);
  cbf_test::Model m;
  cbf_test::refresh(m);
  m.mass(0, 0) = NAN;
  EXPECT_EQ(builder.build(m.view(), .001, 1).status, cbf::Status::invalid_model);
  EXPECT_GT(builder.model_us, 0.);
  EXPECT_EQ(builder.kinematics_us, 0.);
  EXPECT_EQ(builder.constraints_us, 0.);
}
