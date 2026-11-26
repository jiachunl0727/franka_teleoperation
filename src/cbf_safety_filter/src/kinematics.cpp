#include "cbf_safety_filter/kinematics.hpp"
namespace cbf_safety_filter {
PointKinematicsWorkspace::PointKinematicsWorkspace(const ModelView& m,
                                                   const Eigen::Affine3d& world) noexcept
    : dq_(m.dq) {
  std::array<Eigen::Vector3d, kFrames> velocity;
  for (int f = 0; f < kFrames; ++f) {
    poses_[f] = world * m.poses[f];
    jacobians_[f].topRows<3>().noalias() = world.linear() * m.jacobians[f].topRows<3>();
    jacobians_[f].bottomRows<3>().noalias() = world.linear() * m.jacobians[f].bottomRows<3>();
    velocity[f].noalias() = jacobians_[f].topRows<3>() * dq_;
    omega_[f].noalias() = jacobians_[f].bottomRows<3>() * dq_;
  }
  std::array<Eigen::Vector3d, 7> axis_dot;
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  for (int j = 0; j < 7; ++j) {
    const Eigen::Vector3d axis = poses_[j].linear().col(2);
    axis_dot[j] = angular.cross(axis);
    angular += axis * dq_[j];
  }
  for (int f = 0; f < kFrames; ++f) {
    angular_bias_[f].setZero();
    origin_bias_[f].setZero();
    for (int j = 0; j <= std::min(f, 6); ++j) {
      const Eigen::Vector3d axis = poses_[j].linear().col(2);
      angular_bias_[f] += axis_dot[j] * dq_[j];
      origin_bias_[f] += (axis_dot[j].cross(poses_[f].translation() - poses_[j].translation()) +
                          axis.cross(velocity[f] - velocity[j])) *
                         dq_[j];
    }
  }
}
PointKinematics PointKinematicsWorkspace::evaluate(const PointDefinition& point) const noexcept {
  PointKinematics out;
  if (point.frame < 0)
    return out;  // Fixed WORLD points are handled by the caller.
  const int f = point.frame;
  const Eigen::Vector3d offset = poses_[f].linear() * point.offset;
  out.position = poses_[f].translation() + offset;
  for (int j = 0; j < 7; ++j)
    out.jacobian.col(j) =
        jacobians_[f].col(j).head<3>() + jacobians_[f].col(j).tail<3>().cross(offset);
  out.velocity.noalias() = out.jacobian * dq_;
  out.acceleration_bias =
      origin_bias_[f] + angular_bias_[f].cross(offset) + omega_[f].cross(omega_[f].cross(offset));
  return out;
}
PointKinematics point_kinematics(const ModelView& model, const PointDefinition& point,
                                 const Eigen::Affine3d& world) noexcept {
  PointKinematics out;
  const auto& frame = model.poses[point.frame];
  const Eigen::Vector3d offset = frame.linear() * point.offset;
  out.position = world * (frame.translation() + offset);
  const auto& spatial = model.jacobians[point.frame];
  for (int j = 0; j < 7; ++j)
    out.jacobian.col(j) =
        world.linear() * (spatial.col(j).head<3>() + spatial.col(j).tail<3>().cross(offset));
  out.velocity.noalias() = out.jacobian * model.dq;
  Eigen::Vector3d previous_angular = Eigen::Vector3d::Zero();
  for (int j = 0; j <= std::min(point.frame, 6); ++j) {
    const Eigen::Vector3d axis = world.linear() * model.poses[j].linear().col(2);
    const Eigen::Vector3d origin = world * model.poses[j].translation();
    const Eigen::Vector3d origin_velocity =
        world.linear() * (model.jacobians[j].topRows<3>() * model.dq);
    const Eigen::Vector3d axis_dot = previous_angular.cross(axis);
    out.acceleration_bias +=
        (axis_dot.cross(out.position - origin) + axis.cross(out.velocity - origin_velocity)) *
        model.dq[j];
    previous_angular += axis * model.dq[j];
  }
  return out;
}
}  // namespace cbf_safety_filter

namespace cbf_safety_filter {
Vec6 ReferenceNormals::project(Vec6 step) const noexcept {
  if (!step.allFinite())
    return Vec6::Zero();
  for (int sweep = 0; sweep < 12; ++sweep)
    for (int i = 0; i < count; ++i) {
      const double v = normals[i].dot(step), n = normals[i].squaredNorm();
      if (v < 0 && n > 1e-16)
        step -= normals[i] * (v / n);
    }
  // Opposing/nonorthogonal normals: never return an unchecked projected step.
  for (int i = 0; i < count; ++i)
    if (normals[i].dot(step) < -1e-10)
      return Vec6::Zero();
  return step;
}
BodySphere position_body_volume(const BodySphere& definition, const ModelView& m,
                                const Configuration& c) noexcept {
  auto volume = definition;
  if (definition.opening_axis.isZero(0))
    return volume;
  constexpr double full_opening = .08;
  constexpr uint64_t max_age_ns = 30000000;
  // The timestamp uses the local monotonic clock. Cover motion until the next
  // sample and during the lifetime of a peer snapshot. The speed bound is for
  // one finger. At startup or on stale/invalid feedback use the full range.
  double low = 0, high = full_opening;
  const auto& sample = m.finger_opening;
  if (sample.sample_ns && sample.sample_ns <= m.timestamp_ns &&
      m.timestamp_ns - sample.sample_ns <= max_age_ns && std::isfinite(sample.width) &&
      sample.width >= 0 && sample.width <= full_opening) {
    const double horizon = (m.timestamp_ns - sample.sample_ns) * 1e-9 +
                           std::max(.002, c.inter_arm.enabled ? c.peer_max_age_us * 1e-6 : 0.);
    const double width_error = .001 + 2 * .2 * horizon;
    low = std::max(0., sample.width - width_error);
    high = std::min(full_opening, sample.width + width_error);
  }
  volume.offset += definition.opening_axis * ((low + high) * .25);
  volume.radius += (high - low) * .25;
  return volume;
}

PeerSnapshot collision_snapshot(const ModelView& m, const Configuration& c) noexcept {
  PeerSnapshot out;
  out.timestamp_ns = m.timestamp_ns;
  out.count = static_cast<int>(c.body_spheres.size());
  out.speed_limit = c.point_speed_limit;
  out.acceleration_limit = c.point_acceleration_limit;
  out.valid = m.q.allFinite() && m.dq.allFinite() && out.count > 0 && out.count <= kMaxPoints;
  if (!out.valid)
    return out;
  for (int f = 0; f < kFrames; ++f) {
    const auto& t = m.poses[f];
    out.valid &=
        t.matrix().allFinite() && m.jacobians[f].allFinite() &&
        (t.linear().transpose() * t.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
        std::abs(t.linear().determinant() - 1) < 1e-8;
  }
  out.valid &= m.mass.allFinite() && m.coriolis.allFinite();
  if (!out.valid)
    return out;
  for (int i = 0; i < out.count; ++i) {
    const auto b = position_body_volume(c.body_spheres[i], m, c);
    auto& s = out.spheres[i];
    s.radius = b.radius;
    s.group = b.frame + 1;
    s.span = c.world_from_base.linear() *
             (b.frame < 0 ? b.span : (m.poses[b.frame].linear() * b.span).eval());
    if (b.frame < 0) {
      s.position = c.world_from_base * b.offset;
      s.velocity.setZero();
    } else {
      const auto p = point_kinematics(m, b, c.world_from_base);
      s.position = p.position;
      s.velocity = p.velocity;
    }
    out.valid &= s.position.allFinite() && s.velocity.allFinite();
  }
  return out;
}
SingularityKinematics singularity_kinematics(const ModelView& m, double length) noexcept {
  SingularityKinematics out;
  using M6 = Eigen::Matrix<double, 6, 6>;
  Jacobian j = m.jacobians[8], jd = Jacobian::Zero(), jdd = Jacobian::Zero();
  const auto ee =
      point_kinematics(m, {8, Eigen::Vector3d::Zero(), .01}, Eigen::Affine3d::Identity());
  std::array<Eigen::Vector3d, 7> axes, origins, adot, addot, ov, oa;
  Eigen::Vector3d omega = Eigen::Vector3d::Zero(), omega_dot = Eigen::Vector3d::Zero();
  for (int k = 0; k < 7; ++k) {
    axes[k] = m.poses[k].linear().col(2);
    origins[k] = m.poses[k].translation();
    const auto p =
        point_kinematics(m, {k, Eigen::Vector3d::Zero(), .01}, Eigen::Affine3d::Identity());
    ov[k] = p.velocity;
    oa[k] = p.acceleration_bias;
    adot[k] = omega.cross(axes[k]);
    addot[k] = omega_dot.cross(axes[k]) + omega.cross(adot[k]);
    omega_dot += adot[k] * m.dq[k];
    omega += axes[k] * m.dq[k];
    const Eigen::Vector3d r = ee.position - origins[k], v = ee.velocity - ov[k];
    jd.col(k).head<3>() = adot[k].cross(r) + axes[k].cross(v);
    jd.col(k).tail<3>() = adot[k];
    jdd.col(k).head<3>() =
        addot[k].cross(r) + 2 * adot[k].cross(v) + axes[k].cross(ee.acceleration_bias - oa[k]);
    jdd.col(k).tail<3>() = addot[k];
  }
  j.topRows<3>() /= length;
  jd.topRows<3>() /= length;
  jdd.topRows<3>() /= length;
  const M6 g = j * j.transpose();
  Eigen::LLT<M6> llt(g);
  if (llt.info() != Eigen::Success || llt.rcond() < 1e-12)
    return out;
  const M6 p = llt.solve(M6::Identity());
  const double t = p.trace();
  if (!p.allFinite() || !std::isfinite(t) || t <= 0)
    return out;
  out.value = 1 / t;
  for (int k = 0; k < 7; ++k) {
    Jacobian dj = Jacobian::Zero();
    for (int a = 0; a < 7; ++a) {
      const Eigen::Vector3d da = k < a ? axes[k].cross(axes[a]).eval() : Eigen::Vector3d::Zero();
      const Eigen::Vector3d dp = m.jacobians[8].col(k).head<3>();
      const Eigen::Vector3d dorig = m.jacobians[a].col(k).head<3>();
      dj.col(a).head<3>() = da.cross(ee.position - origins[a]) + axes[a].cross(dp - dorig);
      dj.col(a).tail<3>() = da;
    }
    dj.topRows<3>() /= length;
    const M6 dg = dj * j.transpose() + j * dj.transpose();
    out.gradient[k] = (p * dg * p).trace() / (t * t);
  }
  const M6 gd = jd * j.transpose() + j * jd.transpose();
  const M6 gdd = jdd * j.transpose() + 2 * jd * jd.transpose() + j * jdd.transpose();
  const double td = -(p * gd * p).trace();
  const double tdd = (2 * p * gd * p * gd * p - p * gdd * p).trace();
  out.bias = 2 * td * td / (t * t * t) - tdd / (t * t);
  out.valid = out.gradient.allFinite() && std::isfinite(out.bias);
  return out;
}
}  // namespace cbf_safety_filter
