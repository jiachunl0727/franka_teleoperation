#include <gtest/gtest.h>

#include <sstream>

#include "cbf_safety_filter/kinematics.hpp"
#include "fixture.hpp"
namespace cbf = cbf_safety_filter;
TEST(Kinematics, PointJacobianAndAnalyticalBiasMatchIndependentCentralDifferences) {
  double max_j = 0, max_bias = 0;
  for (int trial = 0; trial < 15; ++trial) {
    cbf_test::Model m;
    m.q[0] = .1 * trial;
    m.q[2] = -.06 * trial;
    if (trial == 14)
      m.q.setZero();  // Singular chain still has valid derivatives.
    m.dq << .3, -.2, .7, -.4, .6, -.8, .9;
    m.dq *= trial == 0 ? 0. : 1.;
    cbf_test::refresh(m);
    Eigen::Affine3d world =
        Eigen::Translation3d(.1, .2, .3) * Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX());
    for (int frame = 0; frame < 9; ++frame) {
      const cbf::PointDefinition point{frame, {.04, -.03, .06}, .02};
      const auto actual = cbf::point_kinematics(m.view(), point, world);
      const cbf::PointKinematicsWorkspace workspace(m.view(), world);
      const auto shared = workspace.evaluate(point);
      EXPECT_LT((shared.position - actual.position).norm(), 1e-12);
      EXPECT_LT((shared.velocity - actual.velocity).norm(), 1e-12);
      EXPECT_LT((shared.jacobian - actual.jacobian).norm(), 1e-12);
      EXPECT_LT((shared.acceleration_bias - actual.acceleration_bias).norm(), 1e-12);
      const double eps = 1e-6;
      for (int j = 0; j < 7; ++j) {
        auto plus = m.q, minus = m.q;
        plus[j] += eps;
        minus[j] -= eps;
        const Eigen::Vector3d fd = (world * (cbf_test::forward(plus)[frame] * point.offset) -
                                    world * (cbf_test::forward(minus)[frame] * point.offset)) /
                                   (2 * eps);
        max_j = std::max(max_j, (fd - actual.jacobian.col(j)).norm());
      }
      auto plus = m, minus = m;
      plus.q += eps * m.dq;
      minus.q -= eps * m.dq;
      cbf_test::refresh(plus);
      cbf_test::refresh(minus);
      const cbf::PointJacobian jp = cbf::point_kinematics(plus.view(), point, world).jacobian;
      const cbf::PointJacobian jm = cbf::point_kinematics(minus.view(), point, world).jacobian;
      const Eigen::Vector3d bias = (jp - jm) * m.dq / (2 * eps);
      max_bias = std::max(max_bias, (bias - actual.acceleration_bias).norm());
    }
  }
  EXPECT_LT(max_j, 1e-8);
  EXPECT_LT(max_bias, 1e-8);
  std::ostringstream j_value, bias_value;
  j_value.precision(17);
  bias_value.precision(17);
  j_value << max_j;
  bias_value << max_bias;
  RecordProperty("max_jacobian_error", j_value.str());
  RecordProperty("max_bias_error", bias_value.str());
}
