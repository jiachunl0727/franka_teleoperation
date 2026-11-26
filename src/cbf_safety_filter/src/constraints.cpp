#include "cbf_safety_filter/constraints.hpp"

#include <chrono>
#include <limits>
#include <optional>
namespace cbf_safety_filter {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point from) {
  return std::chrono::duration<double, std::micro>(Clock::now() - from).count();
}
// Attribute partial work on every exit, including model/domain/peer failures.
// Adjacent phases share one boundary timestamp; no allocation or I/O.
struct PhaseTimer {
  double* active;
  Clock::time_point start;
  ~PhaseTimer() { *active = elapsed(start); }
  void next(double& phase) noexcept {
    const auto now = Clock::now();
    *active = std::chrono::duration<double, std::micro>(now - start).count();
    active = &phase;
    start = now;
  }
};
struct GroupBounds {
  int count{0};
  bool finite{true};
  Eigen::Vector3d low, high, velocity_low, velocity_high;
  double radius{0}, map_acc{0}, drift_acc{0};
  void include(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double r, double map = 0,
               double drift = 0) noexcept {
    finite &= p.allFinite() && v.allFinite() && std::isfinite(r) && std::isfinite(map) &&
              std::isfinite(drift);
    if (!count++) {
      low = high = p;
      velocity_low = velocity_high = v;
    } else {
      low = low.cwiseMin(p);
      high = high.cwiseMax(p);
      velocity_low = velocity_low.cwiseMin(v);
      velocity_high = velocity_high.cwiseMax(v);
    }
    radius = std::max(radius, r);
    map_acc = std::max(map_acc, map);
    drift_acc = std::max(drift_acc, drift);
  }
};
// AABB bounds apply to every constituent pair, including velocity and full
// age+period uncertainty. This proves the existing per-pair sufficient test for
// the entire group; proximity alone can never remove a torque/reference row.
bool redundant_group(const GroupBounds& a, const GroupBounds& b, const FamilyConfig& c, double band,
                     double ep, double ev, double peer_acc) noexcept {
  if (!a.count || !b.count || !a.finite || !b.finite)
    return false;
  const double dmin = (a.low - b.high).cwiseMax(b.low - a.high).cwiseMax(0.).norm();
  const double dmax = (a.high - b.low).cwiseAbs().cwiseMax((a.low - b.high).cwiseAbs()).norm();
  const double vmax = (a.velocity_high - b.velocity_low)
                          .cwiseAbs()
                          .cwiseMax((a.velocity_low - b.velocity_high).cwiseAbs())
                          .norm();
  const double radius = a.radius + b.radius + c.clearance;
  const double h = std::pow(std::max(0., dmin - ep), 2) - radius * radius;
  const double hd = -2 * dmax * vmax - 2 * (ep * vmax + (dmax + ep) * ev);
  const double bias =
      -2 * dmax * a.drift_acc - 2 * ep * (a.map_acc + a.drift_acc) - 2 * (dmax + ep) * peer_acc;
  const double rhs = bias + (c.alpha1 + c.alpha2) * hd + c.alpha1 * c.alpha2 * h;
  const double support = 2 * dmax * a.map_acc;
  const double activation = (radius + band) * (radius + band) - radius * radius;
  const double margin = 1e-8 * (1 + std::abs(rhs) + std::abs(support));
  return std::isfinite(rhs) && std::isfinite(support) && h >= activation + margin &&
         hd + c.alpha1 * h >= margin && support <= rhs - margin;
}
// Closest-point separation for two closed segments, including point segments.
Eigen::Vector3d segment_separation(const Eigen::Vector3d& p, const Eigen::Vector3d& u,
                                   const Eigen::Vector3d& q, const Eigen::Vector3d& v) noexcept {
  const Eigen::Vector3d d = p - q;
  const double a = u.squaredNorm(), e = v.squaredNorm(), b = u.dot(v);
  const double c = u.dot(d), f = v.dot(d);
  constexpr double eps = 1e-20;
  double x = 0, y = 0;
  if (a > eps && e > eps) {
    const double determinant = u.cross(v).squaredNorm();
    if (determinant > eps * a * e)
      x = std::clamp((b * f - c * e) / determinant, 0., 1.);
    y = (b * x + f) / e;
    if (y < 0 || y > 1) {
      y = std::clamp(y, 0., 1.);
      x = std::clamp((b * y - c) / a, 0., 1.);
    }
  } else if (a > eps)
    x = std::clamp(-c / a, 0., 1.);
  else if (e > eps)
    y = std::clamp(f / e, 0., 1.);
  return d + x * u - y * v;
}

struct GeometryPruning {
  bool spheres{false}, capsules{false};
};

bool redundant_capsule_groups(const GroupBounds& a, const GroupBounds& b,
                              const FamilyConfig& c) noexcept {
  if (!a.count || !b.count || !a.finite || !b.finite)
    return false;
  const double h = (a.low - b.high).cwiseMax(b.low - a.high).cwiseMax(0.).norm() - a.radius -
                   b.radius - c.clearance;
  const double speed = (a.velocity_high - b.velocity_low)
                           .cwiseAbs()
                           .cwiseMax((a.velocity_low - b.velocity_high).cwiseAbs())
                           .norm();
  const double drift = a.drift_acc + b.drift_acc;
  return h > 0 && c.alpha1 * h > speed + 1e-9 &&
         c.alpha1 * c.alpha2 * h >
             (c.alpha1 + c.alpha2) * speed + drift + a.map_acc + b.map_acc + 1e-9;
}

GeometryPruning redundant_geometry_groups(const GroupBounds& own, const GroupBounds& other,
                                          const FamilyConfig& c, double band) noexcept {
  if (!own.count || !other.count)
    return {};
  auto combined = own;
  combined.map_acc += other.map_acc;
  combined.drift_acc += other.drift_acc;
  return {redundant_group(combined, other, c, band, 0, 0, 0),
          redundant_capsule_groups(own, other, c)};
}

}  // namespace
ConstraintBuilder::ConstraintBuilder(const Configuration& config, bool group_pruning)
    : config_(config), group_pruning_(group_pruning) {
  if (!config.self_collision.enabled)
    return;
  const int n = config.body_spheres.size();
  self_pairs_.reserve(n * (n - 1) / 2);
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      const auto& a = config.body_spheres[i];
      const auto& b = config.body_spheres[j];
      bool excluded = a.group == b.group;
      for (auto e : config.self_exclusions)
        excluded |= (a.group == e.first && b.group == e.second) ||
                    (b.group == e.first && a.group == e.second);
      if (!excluded)
        self_pairs_.emplace_back(i, j);
    }
}
namespace {
void recovery_barrier(double& bound, double& psi, double h, Family family,
                      const FamilyConfig& c) noexcept {
  if (h >= 0)
    return;
  const bool first_order = family == Family::velocity || family == Family::peer_motion;
  // Do not demand autonomous return to a safe position. Outside the boundary,
  // require braking/non-worsening and let the operator choose the retreat.
  bound -= (first_order ? c.alpha1 : c.alpha1 * c.alpha2) * h;
  if (!first_order)
    psi -= c.alpha1 * h;
}
}  // namespace
bool ConstraintBuilder::add(const Vec7& g, double bound, Family family, const FamilyConfig& c,
                            double h, double psi, bool recovery_prepared) noexcept {
  if (!recovery_prepared)
    recovery_barrier(bound, psi, h, family, c);
  if (!g.allFinite() || !std::isfinite(bound) || !std::isfinite(h) || !std::isfinite(psi)) {
    set_.failure = {true, set_.count, {g, bound, family, h, psi}};
    set_.status = Status::invalid_model;
    return false;
  }
  if (set_.count >= config_.max_constraints) {
    set_.failure = {true, set_.count, {g, bound, family, h, psi}};
    set_.status = Status::capacity_exceeded;
    return false;
  }
  const int index = set_.count;
  set_.requested_bounds[index] = bound;
  set_.outside_domain_rows += h < -1e-10 || psi < -1e-10;
  const double anchored = g.dot(set_.braking_anchor);
  if (anchored > bound) {
    ++set_.recovery_rows;
    bound = anchored;
  }
  set_.rows[set_.count++] = {g, bound, family, h, psi};
  if (family == Family::position || family == Family::velocity ||
      family == Family::acceleration) {
    const Vec6 n = task_inverse_.transpose() * (-state_mass_ * g);
    if (n.norm() > 1e-8) {
      set_.rows[index].reference_normal = n.normalized();
      if ((h < 0 || psi < 0) && reference_normals.count < kMaxRows)
        reference_normals.normals[reference_normals.count++] = n.normalized();
    }
  }
  return true;
}
const ConstraintSet& ConstraintBuilder::build(const ModelView& m, double dt, uint64_t now,
                                              const TorqueBounds* bounds, const PeerSnapshot* peer,
                                              const PeerExchange* exchange,
                                              bool peer_avoidance) noexcept {
  const auto start = Clock::now();
  set_.count = 0;
  set_.recovery_rows = set_.outside_domain_rows = 0;
  set_.failure = {};
  reference_normals.count = 0;
  group_pruned_pairs = evaluated_peer_pairs = 0;
  set_.status = Status::solved_hard;
  model_us = kinematics_us = constraints_us = 0;
  PhaseTimer timing{&model_us, start};
  if (!m.q.allFinite() || !m.dq.allFinite() || !m.mass.allFinite() || !m.coriolis.allFinite() ||
      !std::isfinite(dt) || dt <= 0 || dt > .02 || m.timestamp_ns == 0 || now < m.timestamp_ns ||
      (now - m.timestamp_ns) * 1e-3 > config_.max_model_age_us ||
      !m.mass.isApprox(m.mass.transpose(), 1e-9)) {
    set_.status = Status::invalid_model;
    return set_;
  }
  for (int f = 0; f < kFrames; ++f) {
    const auto& pose = m.poses[f];
    if (!pose.matrix().allFinite() || !m.jacobians[f].allFinite() ||
        !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) ||
        !(pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
        std::abs(pose.linear().determinant() - 1) > 1e-8) {
      set_.status = Status::invalid_model;
      return set_;
    }
  }
  mass_.compute(m.mass);
  if (mass_.info() != Eigen::Success || mass_.rcond() < 1e-10) {
    set_.status = Status::invalid_model;
    return set_;
  }
  state_mass_ = m.mass;
  if (!bounds || !bounds->lower.allFinite() || !bounds->upper.allFinite() ||
      (bounds->lower.array() > bounds->upper.array()).any()) {
    set_.status = Status::invalid_model;
    return set_;
  }
  const Vec7 brake_acceleration = (-config_.recovery_braking_gain * m.dq)
                                      .cwiseMax(-config_.acceleration_limits)
                                      .cwiseMin(config_.acceleration_limits);
  set_.braking_anchor =
      (m.coriolis + m.mass * brake_acceleration).cwiseMax(bounds->lower).cwiseMin(bounds->upper);
  const Vec7 drift = -mass_.solve(m.coriolis);
  {
    const Jacobian& j = m.jacobians[8];
    Eigen::Matrix<double, 6, 6> a = j * j.transpose();
    a.diagonal().array() += 1e-8;
    task_inverse_ = j.transpose() * a.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
  }
  timing.next(kinematics_us);
  std::optional<PointKinematicsWorkspace> workspace;
  if (config_.table.enabled || config_.obstacles.enabled || config_.self_collision.enabled ||
      config_.inter_arm.enabled)
    workspace.emplace(m, config_.world_from_base);
  // Publish the very same point values that local constraints consume. No
  // separate point-derivative pass and no cross-cycle geometry cache.
  if (config_.self_collision.enabled || config_.inter_arm.enabled ||
      (config_.table.enabled && config_.uses_body_volumes))
    for (size_t i = 0; i < config_.body_spheres.size(); ++i) {
      const auto b = position_body_volume(config_.body_spheres[i], m, config_);
      body_radii_[i] = b.radius;
      const auto evaluate_body = [&](const Eigen::Vector3d& offset) {
        if (b.frame >= 0)
          return workspace->evaluate({b.frame, offset, b.radius});
        PointKinematics fixed;
        fixed.position = config_.world_from_base * offset;
        return fixed;
      };
      bodies_[i] = evaluate_body(b.offset);
      if (config_.uses_body_volumes)
        body_ends_[i] = b.span.isZero(0) ? bodies_[i] : evaluate_body(b.offset + b.span);
    }
  if (config_.inter_arm.enabled && exchange && exchange->publish_read) {
    PeerSnapshot own;
    own.timestamp_ns = m.timestamp_ns;
    own.count = static_cast<int>(config_.body_spheres.size());
    own.speed_limit = config_.point_speed_limit;
    own.acceleration_limit = config_.point_acceleration_limit;
    own.valid = own.count > 0 && own.count <= kMaxPoints;
    for (int i = 0; i < own.count; ++i) {
      const auto& p = bodies_[i];
      own.spheres[i] = {p.position, p.velocity, body_radii_[i], config_.body_spheres[i].frame + 1,
                        config_.uses_body_volumes ? (body_ends_[i].position - p.position).eval()
                                                   : Eigen::Vector3d::Zero().eval()};
      own.valid &= p.position.allFinite() && p.velocity.allFinite() &&
                   own.spheres[i].span.allFinite();
    }
    peer = exchange->publish_read(exchange->context, own);
    now = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
              .count();
  }
  int point_count = 0;
  const auto evaluate = [&](const PointDefinition& definition) {
    auto& p = points_[point_count++];
    p = workspace->evaluate(definition);
    p.torque_map = mass_.solve(p.jacobian.transpose()).transpose();
    p.drift_acceleration = p.acceleration_bias + p.jacobian * drift;
  };
  if (config_.table.enabled && !config_.uses_body_volumes)
    for (const auto& c : config_.capsules) {
      evaluate({c.frame, c.a, c.radius});
      evaluate({c.frame, c.b, c.radius});
    }
  const int obstacle_start = point_count;
  if (config_.obstacles.enabled)
    for (const auto& p : config_.points)
      evaluate(p);
  if (config_.self_collision.enabled || config_.inter_arm.enabled ||
      (config_.table.enabled && config_.uses_body_volumes))
    for (size_t i = 0; i < config_.body_spheres.size(); ++i) {
      const auto dynamics = [&](PointKinematics& p) {
        p.torque_map = mass_.solve(p.jacobian.transpose()).transpose();
        p.drift_acceleration = p.acceleration_bias + p.jacobian * drift;
      };
      dynamics(bodies_[i]);
      if (config_.uses_body_volumes)
        dynamics(body_ends_[i]);
    }
  timing.next(constraints_us);
  for (int j = 0; j < 7; ++j) {
    const Vec7 map = mass_.solve(Vec7::Unit(j));
    if (config_.position.enabled) {
      const auto& c = config_.position;
      for (double sign : {-1., 1.}) {
        const double h = sign > 0 ? m.q[j] - config_.position_lower_limits[j] - c.margin
                                   : config_.position_upper_limits[j] - c.margin - m.q[j];
        const double hdot = sign * m.dq[j];
        if (!add(-sign * map,
                 sign * drift[j] + (c.alpha1 + c.alpha2) * hdot + c.alpha1 * c.alpha2 * h,
                 Family::position, c, h, hdot + c.alpha1 * h))
          return set_;
      }
    }
    if (config_.velocity.enabled) {
      const auto& c = config_.velocity;
      for (double sign : {-1., 1.}) {
        const double h = config_.velocity_limits[j] - c.margin + sign * m.dq[j];
        if (!add(-sign * map, sign * drift[j] + c.alpha1 * h, Family::velocity, c, h))
          return set_;
      }
    }
    if (config_.acceleration.enabled)
      for (double sign : {-1., 1.})
        if (!add(sign * map, config_.acceleration_limits[j] - sign * drift[j], Family::acceleration,
                 config_.acceleration))
          return set_;
  }
  int index = 0;
  if (config_.table.enabled) {
    const auto table_endpoint = [&](const PointKinematics& p, double radius) {
      const auto& c = config_.table;
      const double h = p.position.z() - c.height - radius - c.clearance;
      const double hdot = p.velocity.z();
      return collision_row(
          -p.torque_map.row(2).transpose(),
          p.drift_acceleration.z() + (c.alpha1 + c.alpha2) * hdot + c.alpha1 * c.alpha2 * h,
          Family::table, c, h, hdot + c.alpha1 * h, p.jacobian.row(2).transpose(), nullptr);
    };
    if (config_.uses_body_volumes) {
      for (size_t i = 0; i < config_.body_spheres.size(); ++i)
        if (const auto& volume = config_.body_spheres[i]; volume.used_for_capsule_table() &&
            (!table_endpoint(bodies_[i], body_radii_[i]) ||
             (!volume.span.isZero(0) && !table_endpoint(body_ends_[i], body_radii_[i]))))
          return set_;
    } else {
      for (const auto& capsule : config_.capsules)
        for (int endpoint = 0; endpoint < 2; ++endpoint)
          if (!table_endpoint(points_[index++], capsule.radius))
            return set_;
    }
  }
  if (config_.obstacles.enabled)
    for (auto pair : config_.pairs) {
      const auto& p = points_[obstacle_start + pair.point];
      const auto& obstacle = config_.obstacle_spheres[pair.obstacle];
      const auto& c = config_.obstacles;
      const Eigen::Vector3d d = p.position - obstacle.center;
      const double radius = config_.points[pair.point].radius + obstacle.radius + c.clearance;
      const double h = d.squaredNorm() - radius * radius, hdot = 2 * d.dot(p.velocity);
      const double bias = 2 * p.velocity.squaredNorm() + 2 * d.dot(p.drift_acceleration);
      if (!collision_row(-2 * p.torque_map.transpose() * d,
                         bias + (c.alpha1 + c.alpha2) * hdot + c.alpha1 * c.alpha2 * h,
                         Family::obstacle, c, h, hdot + c.alpha1 * h,
                         2 * p.jacobian.transpose() * d, bounds))
        return set_;
    }
  std::array<GroupBounds, kFrames + 1> capsule_groups{};
  if (config_.uses_body_volumes && (config_.self_collision.enabled || config_.inter_arm.enabled)) {
    const Vec7 boxmax = bounds->lower.cwiseAbs().cwiseMax(bounds->upper.cwiseAbs());
    for (size_t i = 0; i < config_.body_spheres.size(); ++i)
      for (const auto* p : {&bodies_[i], &body_ends_[i]}) {
        double map = 0;
        for (int j = 0; j < 7; ++j)
          map += p->torque_map.col(j).norm() * boxmax[j];
        capsule_groups[config_.body_spheres[i].frame + 1].include(
            p->position, p->velocity, body_radii_[i], map, p->drift_acceleration.norm());
      }
  }
  if (config_.self_collision.enabled) {
    const auto& c = config_.self_collision;
    std::array<std::array<GeometryPruning, kFrames + 1>, kFrames + 1> skip{};
    if (config_.uses_body_volumes && group_pruning_)
      for (int i = 0; i <= kFrames; ++i)
        for (int j = i; j <= kFrames; ++j)
          skip[i][j] = skip[j][i] = redundant_geometry_groups(capsule_groups[i], capsule_groups[j],
                                                              c, config_.boundary_activation);
    for (auto indices : self_pairs_) {
      const int i = indices.first, j = indices.second;
      const auto& a = config_.body_spheres[i];
      const auto& b = config_.body_spheres[j];
      const auto& p = bodies_[i];
      const auto& q = bodies_[j];
      const bool have_span = config_.uses_body_volumes && (!a.span.isZero(0) || !b.span.isZero(0));
      const auto& pruning = skip[a.frame + 1][b.frame + 1];
      if (have_span ? pruning.capsules : pruning.spheres)
        continue;
      if (have_span) {
        if (!capsule_row(p, body_ends_[i], q, body_ends_[j],
                         body_radii_[i] + body_radii_[j] + c.clearance, Family::self_collision, c,
                         bounds))
          return set_;
        continue;
      }
      const Eigen::Vector3d d = p.position - q.position, v = p.velocity - q.velocity;
      const PointJacobian map = p.torque_map - q.torque_map;
      const double radius = body_radii_[i] + body_radii_[j] + c.clearance;
      const double h = d.squaredNorm() - radius * radius, hd = 2 * d.dot(v);
      const double bias =
          2 * v.squaredNorm() + 2 * d.dot(p.drift_acceleration - q.drift_acceleration);
      const Vec7 gradient = 2 * (p.jacobian - q.jacobian).transpose() * d;
      if (!collision_row(
              -2 * map.transpose() * d, bias + (c.alpha1 + c.alpha2) * hd + c.alpha1 * c.alpha2 * h,
              Family::self_collision, c, h, hd + c.alpha1 * h, gradient, bounds))
        return set_;
    }
  }
  if (config_.inter_arm.enabled) {
    const auto& c = config_.inter_arm;
    // Every enabled participant enforces these Cartesian point speed and acceleration
    // boxes. Thus the peer's advertised Euclidean bounds have an explicit producer.
    for (size_t i = 0; i < config_.body_spheres.size(); ++i) {
      const int ends =
          config_.uses_body_volumes && !config_.body_spheres[i].span.isZero(0) ? 2 : 1;
      for (int endpoint = 0; endpoint < ends; ++endpoint) {
        const auto& p = endpoint ? body_ends_[i] : bodies_[i];
        for (int axis = 0; axis < 3; ++axis)
          for (double sign : {-1., 1.}) {
            const Vec7 g = sign * p.torque_map.row(axis).transpose();
            const Vec7 reference_gradient(-sign * p.jacobian.row(axis).transpose());
            const double a_bound = config_.point_acceleration_limit / std::sqrt(3.) -
                                   sign * p.drift_acceleration[axis];
            if (!collision_row(g, a_bound, Family::peer_motion, c, 1, 1, reference_gradient,
                               bounds))
              return set_;
            const double h = config_.point_speed_limit / std::sqrt(3.) - sign * p.velocity[axis];
            if (!collision_row(g, c.alpha1 * h - sign * p.drift_acceleration[axis],
                               Family::peer_motion, c, h, 1, reference_gradient, bounds))
              return set_;
          }
      }
    }
  }
  if (config_.inter_arm.enabled) {
    const auto& c = config_.inter_arm;
    if (exchange && exchange->read_latest) {
      peer = exchange->read_latest(exchange->context);
      now = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count();
    }
    const double age = peer && now >= peer->timestamp_ns ? (now - peer->timestamp_ns) * 1e-9 : -1;
    const double skew =
        peer ? std::abs(double(m.timestamp_ns) - double(peer->timestamp_ns)) * 1e-3 : 1e30;
    if (!peer || !peer->valid || peer->count <= 0 || peer->count > kMaxPoints ||
        peer->timestamp_ns == 0 || age < 0 || !std::isfinite(peer->speed_limit) ||
        !std::isfinite(peer->acceleration_limit) || peer->speed_limit <= 0 ||
        peer->acceleration_limit <= 0 || !bounds || !bounds->lower.allFinite() ||
        !bounds->upper.allFinite()) {
      set_.status = Status::invalid_model;
      return set_;
    }
    if (age * 1e6 > config_.peer_max_age_us || skew > config_.peer_max_skew_us) {
      set_.status = Status::stale_peer;
      return set_;
    }
    for (int j = 0; j < peer->count; ++j) {
      const auto& q = peer->spheres[j];
      if (!q.position.allFinite() || !q.velocity.allFinite() || !q.span.allFinite() ||
          (!config_.uses_body_volumes && !q.span.isZero(0)) || !std::isfinite(q.radius) ||
          q.radius <= 0) {
        set_.status = Status::invalid_model;
        return set_;
      }
    }
    if (config_.uses_body_volumes && peer_avoidance) {
      std::array<GroupBounds, kFrames + 1> peer_groups{};
      std::array<std::array<GeometryPruning, kFrames + 1>, kFrames + 1> skip{};
      for (int j = 0; j < peer->count; ++j) {
        const auto& q = peer->spheres[j];
        const int group = std::clamp(q.group, 0, kFrames);
        peer_groups[group].include(q.position, Eigen::Vector3d::Zero(), q.radius);
        peer_groups[group].include(q.position + q.span, Eigen::Vector3d::Zero(), q.radius);
      }
      if (group_pruning_)
        for (int i = 0; i <= kFrames; ++i)
          for (int j = 0; j <= kFrames; ++j)
            skip[i][j] = redundant_geometry_groups(capsule_groups[i], peer_groups[j], c,
                                                   config_.inter_arm_activation());
      for (size_t i = 0; i < config_.body_spheres.size(); ++i)
        for (int j = 0; j < peer->count; ++j) {
          const auto& remote = peer->spheres[j];
          const auto& pruning =
              skip[config_.body_spheres[i].frame + 1][std::clamp(remote.group, 0, kFrames)];
          const bool have_span = !config_.body_spheres[i].span.isZero(0) || !remote.span.isZero(0);
          if (have_span ? pruning.capsules : pruning.spheres) {
            ++group_pruned_pairs;
            continue;
          }
          ++evaluated_peer_pairs;
          PointKinematics q, end;
          q.position = remote.position;
          end.position = remote.position + remote.span;
          if (!capsule_row(bodies_[i], body_ends_[i], q, end,
                           body_radii_[i] + remote.radius + c.clearance, Family::inter_arm, c,
                           bounds))
            return set_;
        }
    } else {
      if (peer_avoidance) {
        std::array<GroupBounds, kFrames + 1> own_groups{}, peer_groups{};
        std::array<std::array<bool, kFrames + 1>, kFrames + 1> skip{};
        if (group_pruning_) {
          const Vec7 boxmax = bounds->lower.cwiseAbs().cwiseMax(bounds->upper.cwiseAbs());
          for (size_t i = 0; i < config_.body_spheres.size(); ++i) {
            const auto& p = bodies_[i];
            double map = 0;
            for (int j = 0; j < 7; ++j)
              map += p.torque_map.col(j).norm() * boxmax[j];
            own_groups[config_.body_spheres[i].frame + 1].include(p.position, p.velocity,
                                                                  config_.body_spheres[i].radius,
                                                                  map, p.drift_acceleration.norm());
          }
          for (int j = 0; j < peer->count; ++j) {
            const auto& p = peer->spheres[j];
            const int group = p.group >= 0 && p.group <= kFrames ? p.group : 0;
            peer_groups[group].include(p.position, Eigen::Vector3d::Zero(), p.radius);
          }
          for (int i = 0; i <= kFrames; ++i)
            for (int j = 0; j <= kFrames; ++j)
              skip[i][j] = redundant_group(own_groups[i], peer_groups[j], c,
                                           config_.inter_arm_activation(), 0, 0, 0);
        }
        for (size_t i = 0; i < config_.body_spheres.size(); ++i) {
          const auto& p = bodies_[i];
          for (int j = 0; j < peer->count; ++j) {
            const auto& q = peer->spheres[j];
            const int group = q.group >= 0 && q.group <= kFrames ? q.group : 0;
            if (skip[config_.body_spheres[i].frame + 1][group]) {
              ++group_pruned_pairs;
              continue;
            }
            ++evaluated_peer_pairs;
            const Eigen::Vector3d d = p.position - q.position;
            const double radius = config_.body_spheres[i].radius + q.radius + c.clearance;
            const double h = d.squaredNorm() - radius * radius;
            const double hd = 2 * d.dot(p.velocity);
            const double bias = 2 * p.velocity.squaredNorm() + 2 * d.dot(p.drift_acceleration);
            if (!collision_row(-2 * p.torque_map.transpose() * d,
                               bias + (c.alpha1 + c.alpha2) * hd + c.alpha1 * c.alpha2 * h,
                               Family::inter_arm, c, h, hd + c.alpha1 * h,
                               2 * p.jacobian.transpose() * d, bounds))
              return set_;
          }
        }
      }
    }
  }

  if (config_.singularity.enabled) {
    const auto& c = config_.singularity;
    const auto k = singularity_kinematics(m, config_.singularity_length);
    if (!k.valid) {
      Row failed;
      failed.family = Family::singularity;
      failed.h = failed.psi = std::numeric_limits<double>::quiet_NaN();
      set_.failure = {true, set_.count, failed};
      set_.status = Status::invalid_domain;
      return set_;
    }
    const double h = k.value - config_.singularity_minimum * config_.singularity_minimum;
    const double hd = k.gradient.dot(m.dq);
    const double bias = k.bias + k.gradient.dot(drift);
    if (!collision_row(-mass_.solve(k.gradient),
                       bias + (c.alpha1 + c.alpha2) * hd + c.alpha1 * c.alpha2 * h,
                       Family::singularity, c, h, hd + c.alpha1 * h, k.gradient, bounds))
      return set_;
  }
  return set_;
}
bool ConstraintBuilder::capsule_row(const PointKinematics& a, const PointKinematics& b,
                                    const PointKinematics& p, const PointKinematics& q,
                                    double radius, Family family, const FamilyConfig& c,
                                    const TorqueBounds* bounds) noexcept {
  const Eigen::Vector3d u = b.position - a.position, v = q.position - p.position;
  if (u.isZero(0) && v.isZero(0)) {
    const Eigen::Vector3d d = a.position - p.position, velocity = a.velocity - p.velocity;
    const double h = d.squaredNorm() - radius * radius, hd = 2 * d.dot(velocity);
    const double bias =
        2 * (velocity.squaredNorm() + d.dot(a.drift_acceleration - p.drift_acceleration));
    return collision_row(-2 * (a.torque_map - p.torque_map).transpose() * d,
                         bias + (c.alpha1 + c.alpha2) * hd + c.alpha1 * c.alpha2 * h, family, c, h,
                         hd + c.alpha1 * h, 2 * (a.jacobian - p.jacobian).transpose() * d, bounds);
  }
  Eigen::Vector3d normal = segment_separation(a.position, u, p.position, v);
  if (normal.squaredNorm() < 1e-20) {
    // A crossing centerline has no unique outward normal. Choose a separating
    // direction perpendicular to the axes so recovery still has a direction.
    normal = u.cross(v);
    if (normal.squaredNorm() < 1e-20) {
      const Eigen::Vector3d axis = u.squaredNorm() > v.squaredNorm() ? u : v;
      normal = axis.squaredNorm() < 1e-20 ? Eigen::Vector3d::UnitX().eval() : axis.unitOrthogonal();
    }
  }
  normal.normalize();
  // The projected gap lower-bounds separation along this fixed normal.
  // Checking the endpoint combinations covers the whole segments, including
  // switches of closest points and parallel side-on configurations.
  for (const auto* own : {&a, &b}) {
    for (const auto* other : {&p, &q}) {
      const double h = normal.dot(own->position - other->position) - radius;
      const double hd = normal.dot(own->velocity - other->velocity);
      const double bias = normal.dot(own->drift_acceleration - other->drift_acceleration);
      const Vec7 gradient = (own->jacobian - other->jacobian).transpose() * normal;
      if (!collision_row(-(own->torque_map - other->torque_map).transpose() * normal,
                         bias + (c.alpha1 + c.alpha2) * hd + c.alpha1 * c.alpha2 * h, family, c, h,
                         hd + c.alpha1 * h, gradient, bounds))
        return false;
      if (v.isZero(0))
        break;
    }
    if (u.isZero(0))
      break;
  }
  return true;
}
bool ConstraintBuilder::collision_row(const Vec7& g, double bound, Family family,
                                      const FamilyConfig& c, double h, double psi,
                                      const Vec7& gradient, const TorqueBounds* bounds) noexcept {
  recovery_barrier(bound, psi, h, family, c);
  if (!g.allFinite() || !std::isfinite(bound) || !std::isfinite(h) || !std::isfinite(psi)) {
    set_.failure = {true, set_.count, {g, bound, family, h, psi}};
    set_.status = Status::invalid_model;
    return false;
  }
  if ((h < 0 || psi < 0) &&
      reference_normals.count < kMaxRows) {
    Vec6 n = task_inverse_.transpose() * gradient;
    if (n.squaredNorm() > 1e-16)
      reference_normals.normals[reference_normals.count++] = n.normalized();
  }
  // Exact box redundancy test, not a geometric distance heuristic. A removed
  // row is satisfied by every actuator-admissible command for this cycle.
  if (bounds) {
    double worst = 0;
    for (int j = 0; j < 7; ++j)
      worst += g[j] * (g[j] >= 0 ? bounds->upper[j] : bounds->lower[j]);
    if (worst <= bound - 1e-9) {
      set_.outside_domain_rows += h < -1e-10 || psi < -1e-10;
      return true;
    }
  }
  if (!add(g, bound, family, c, h, psi, true))
    return false;
  if (gradient.squaredNorm() > 1e-20) {
    const Vec6 normal = task_inverse_.transpose() * gradient;
    if (normal.norm() > 1e-8)
      set_.rows[set_.count - 1].reference_normal = normal.normalized();
  }
  return true;
}
}  // namespace cbf_safety_filter
