#pragma once
#include "cbf_safety_filter/types.hpp"
namespace cbf_test {
namespace cbf = cbf_safety_filter;
struct Model {
  cbf::Vec7 q{(cbf::Vec7() << 0, -.7854, 0, -2.3562, 0, 1.5708, .7854).finished()},
      dq{cbf::Vec7::Zero()}, coriolis{cbf::Vec7::Zero()};
  cbf::Mat7 mass{cbf::Mat7::Identity()};
  std::array<Eigen::Affine3d, 9> poses;
  std::array<cbf::Jacobian, 9> jacobians;
  cbf::ModelView view() const { return {q, dq, mass, coriolis, poses, jacobians, 1}; }
};
// Independent modified-DH reference for the pinned Panda URDF, not library FK.
inline std::array<Eigen::Affine3d, 9> forward(const cbf::Vec7& q) {
  const double alpha[7] = {0, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2};
  const double a[7] = {0, 0, 0, .0825, -.0825, 0, .088}, d[7] = {.333, 0, .316, 0, .384, 0, 0};
  std::array<Eigen::Affine3d, 9> p;
  Eigen::Affine3d t = Eigen::Affine3d::Identity();
  for (int j = 0; j < 7; ++j) {
    t = t * Eigen::AngleAxisd(alpha[j], Eigen::Vector3d::UnitX()) *
        Eigen::Translation3d(a[j], 0, 0) * Eigen::AngleAxisd(q[j], Eigen::Vector3d::UnitZ()) *
        Eigen::Translation3d(0, 0, d[j]);
    p[j] = t;
  }
  p[7] = t * Eigen::Translation3d(0, 0, .107);
  p[8] = p[7] * Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()) *
         Eigen::Translation3d(0, 0, .1034);
  return p;
}
inline void refresh(Model& m) {
  m.poses = forward(m.q);
  // Geometric reference at frame origins; tests independently differentiate FK.
  for (int f = 0; f < 9; ++f) {
    m.jacobians[f].setZero();
    for (int j = 0; j <= std::min(f, 6); ++j) {
      const Eigen::Vector3d axis = m.poses[j].linear().col(2);
      m.jacobians[f].col(j).head<3>() =
          axis.cross(m.poses[f].translation() - m.poses[j].translation());
      m.jacobians[f].col(j).tail<3>() = axis;
    }
  }
}
}  // namespace cbf_test
