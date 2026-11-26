#include "cbf_safety_filter/configuration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace cbf_safety_filter {
namespace {
void require(bool ok, const std::string& message) {
  if (!ok)
    throw std::invalid_argument(message);
}
YAML::Node merge_document(const YAML::Node& base, const YAML::Node& overlay) {
  if (!base || !base.IsMap() || !overlay.IsMap())
    return YAML::Clone(overlay);
  auto result = YAML::Clone(base);
  for (auto entry : overlay) {
    const auto key = entry.first.as<std::string>();
    result[key] = merge_document(base[key], entry.second);
  }
  return result;
}
void unique_keys(const YAML::Node& node, int depth = 0) {
  require(depth < 64, "CBF YAML nesting too deep");
  if (node.IsMap()) {
    std::set<std::string> seen;
    for (auto entry : node) {
      require(seen.insert(entry.first.as<std::string>()).second, "duplicate CBF key");
      unique_keys(entry.second, depth + 1);
    }
  } else if (node.IsSequence())
    for (auto entry : node)
      unique_keys(entry, depth + 1);
}
YAML::Node load_document(const std::filesystem::path& path, std::set<std::string>& active) {
  const auto canonical = std::filesystem::weakly_canonical(path);
  require(active.size() < 8 && active.insert(canonical.string()).second,
          "cyclic or excessive CBF profile inheritance");
  auto layer = YAML::LoadFile(canonical.string());
  require(layer.IsMap(), "CBF profile must be a mapping");
  unique_keys(layer);
  const YAML::Node view = layer;
  YAML::Node parent(YAML::NodeType::Undefined);
  if (view["extends"]) {
    const auto source = view["extends"].as<std::string>();
    require(!source.empty(), "empty CBF base profile");
    parent = load_document(canonical.parent_path() / source, active);
    layer.remove("extends");
  }
  // Geometry paths belong to the file that declares them, even across folders.
  const auto geometry = view["geometry"];
  if (geometry && geometry.IsMap()) {
    if (geometry["file"] && !geometry["file"].as<std::string>().empty())
      layer["geometry"]["file"] = (canonical.parent_path() / geometry["file"].as<std::string>())
                                      .lexically_normal()
                                      .string();
    if (geometry["files_by_controller"] && geometry["files_by_controller"].IsMap())
      for (auto entry : geometry["files_by_controller"]) {
        if (entry.second.as<std::string>().empty())
          continue;
        layer["geometry"]["files_by_controller"][entry.first.as<std::string>()] =
            (canonical.parent_path() / entry.second.as<std::string>()).lexically_normal().string();
      }
  }
  active.erase(canonical.string());
  return parent ? merge_document(parent, layer) : layer;
}
void keys(const YAML::Node& node, std::initializer_list<const char*> allowed) {
  if (!node)
    return;
  require(node.IsMap(), "CBF configuration requires mappings");
  std::set<std::string> seen;
  for (auto entry : node) {
    const auto key = entry.first.as<std::string>();
    require(seen.insert(key).second, "duplicate CBF key: " + key);
    bool known = false;
    for (const auto* candidate : allowed)
      known |= key == candidate;
    require(known, "unknown CBF key: " + key);
  }
}
bool boolean(const YAML::Node& node, const char* key, bool fallback) {
  if (!node || !node[key])
    return fallback;
  const auto value = node[key].as<std::string>();
  require(value == "true" || value == "false", std::string(key) + " must be true/false");
  return value == "true";
}
template <class T>
T get(const YAML::Node& n, const char* key, T fallback) {
  return n && n[key] ? n[key].as<T>() : fallback;
}
void choice(const YAML::Node& n, const char* key, const char* required) {
  require(n && n[key] && n[key].as<std::string>() == required, std::string("unsupported ") + key);
}
template <int N>
Eigen::Matrix<double, N, 1> vector(const YAML::Node& n) {
  require(n.IsSequence() && n.size() == N, "invalid CBF vector dimension");
  Eigen::Matrix<double, N, 1> result;
  for (int j = 0; j < N; ++j)
    result[j] = n[j].as<double>();
  require(result.allFinite(), "nonfinite CBF vector");
  return result;
}
void family(const YAML::Node& n, FamilyConfig& f, Family kind) {
  if (kind == Family::position)
    keys(n, {"enabled", "lower_limits_rad", "upper_limits_rad", "safety_margin_rad",
             "alpha1", "alpha2"});
  if (kind == Family::velocity)
    keys(n, {"enabled", "maximum_limits_rad_s", "safety_margin_rad_s", "alpha"});
  if (kind == Family::acceleration)
    keys(n, {"enabled", "limits_rad_s2"});
  if (kind == Family::table)
    keys(n, {"enabled", "height_m", "clearance_m", "alpha1", "alpha2"});
  if (kind == Family::obstacle)
    keys(n, {"enabled", "clearance_m", "alpha1", "alpha2"});
  if (kind == Family::self_collision)
    keys(n, {"enabled", "clearance_m", "boundary_activation_m", "alpha1", "alpha2"});
  if (kind == Family::inter_arm)
    keys(n, {"enabled", "clearance_m", "alpha1", "alpha2", "max_peer_age_us", "max_peer_skew_us",
             "point_speed_limit_m_s", "point_acceleration_limit_m_s2", "reference_activation_m",
             "policy"});
  if (kind == Family::singularity)
    keys(n, {"enabled", "alpha1", "alpha2", "characteristic_length_m", "minimum_normalized_sigma"});
  f.enabled = boolean(n, "enabled", f.enabled);
  f.margin =
      get(n, kind == Family::velocity ? "safety_margin_rad_s" : "safety_margin_rad", f.margin);
  f.alpha1 = get(n, kind == Family::velocity ? "alpha" : "alpha1", f.alpha1);
  f.alpha2 = get(n, "alpha2", f.alpha2);
  f.height = get(n, "height_m", f.height);
  f.clearance = get(n, "clearance_m", f.clearance);
}
void geometry(const std::filesystem::path& path, Configuration& c) {
  const auto n = YAML::LoadFile(path.string());
  keys(n, {"version", "world_from_base_xyz_xyzw", "capsules", "points", "obstacles", "pairs",
           "body_spheres", "self_exclusions", "body_volumes"});
  require(get(n, "version", 0) == 1, "unsupported geometry version");
  const auto transform = vector<7>(n["world_from_base_xyz_xyzw"]);
  Eigen::Quaterniond q(transform[6], transform[3], transform[4], transform[5]);
  require(std::abs(q.norm() - 1) < 1e-8, "base transform quaternion must be unit");
  c.world_from_base = Eigen::Translation3d(transform.head<3>()) * q;
  c.uses_body_volumes = static_cast<bool>(n["body_volumes"]);
  require(!c.uses_body_volumes || (!n["capsules"] && !n["body_spheres"]),
          "body volumes already supply both table and body collision geometry");
  for (auto v : n["capsules"]) {
    keys(v, {"frame", "a", "b", "radius_m"});
    c.capsules.push_back(
        {v["frame"].as<int>(), vector<3>(v["a"]), vector<3>(v["b"]), v["radius_m"].as<double>()});
  }
  for (auto v : n["points"]) {
    keys(v, {"frame", "offset", "radius_m"});
    c.points.push_back({v["frame"].as<int>(), vector<3>(v["offset"]), v["radius_m"].as<double>()});
  }
  const auto body = c.uses_body_volumes ? n["body_volumes"] : n["body_spheres"];
  require(!c.uses_body_volumes || (body && body.IsSequence() && body.size()),
          "body_volumes must be a nonempty sequence");
  for (auto v : body) {
    keys(v, {"frame", "offset", "radius_m", "group", "end", "opening_axis"});
    BodySphere b;
    b.frame = v["frame"].as<int>();
    b.offset = vector<3>(v["offset"]);
    b.radius = v["radius_m"].as<double>();
    b.group = v["group"].as<int>();
    if (v["end"])
      b.span = vector<3>(v["end"]) - b.offset;
    if (v["opening_axis"])
      b.opening_axis = vector<3>(v["opening_axis"]);
    require(c.uses_body_volumes || (b.span.isZero(0) && b.opening_axis.isZero(0)),
            "capsule endpoints and finger axes require body_volumes");
    c.body_spheres.push_back(b);
  }
  for (auto v : n["self_exclusions"]) {
    require(v.IsSequence() && v.size() == 2, "invalid self exclusion");
    c.self_exclusions.push_back({v[0].as<int>(), v[1].as<int>()});
  }
  for (auto v : n["obstacles"]) {
    keys(v, {"center", "radius_m"});
    c.obstacle_spheres.push_back({vector<3>(v["center"]), v["radius_m"].as<double>()});
  }
  for (auto v : n["pairs"]) {
    keys(v, {"point", "obstacle"});
    c.pairs.push_back({v["point"].as<int>(), v["obstacle"].as<int>()});
  }
}
}  // namespace
void Configuration::validate() const {
  require(max_constraints > 0 && max_constraints <= kMaxRows,
          "invalid CBF workspace capacity");
  require(weights.allFinite() && (weights.array() > 0).all() &&
              weights.maxCoeff() / weights.minCoeff() <= 1e8,
          "invalid objective weights");
  require(acceleration_limits.allFinite() && (acceleration_limits.array() > 0).all(),
          "invalid acceleration limits");
  require(position_lower_limits.allFinite() && position_upper_limits.allFinite() &&
              (position_lower_limits.array() < position_upper_limits.array()).all(),
          "invalid joint position limits");
  require(velocity_limits.allFinite() && (velocity_limits.array() > 0).all(),
          "invalid joint velocity limits");
  for (double v : {max_filter_time_us, max_model_age_us, max_callback_time_us, osqp_abs, osqp_rel,
                   qpoases_tolerance, piqp_abs, piqp_rel, hard_tolerance})
    require(std::isfinite(v) && v > 0, "CBF tolerances/deadline must be finite and positive");
  for (int n : {osqp_iterations, qpoases_recalculations, piqp_iterations})
    require(n > 0 && n <= 100000, "invalid solver iteration/work-set cap");
  int rows = 0;
  const auto check = [&](const FamilyConfig& f, int count) {
    for (double v : {f.margin, f.clearance})
      require(std::isfinite(v) && v >= 0, "invalid nonnegative family setting");
    for (double v : {f.alpha1, f.alpha2})
      require(std::isfinite(v) && v > 0, "invalid positive family setting");
    require(std::isfinite(f.height), "nonfinite table height");
    if (f.enabled)
      rows += count;
  };
  check(position, 14);
  check(velocity, 14);
  check(acceleration, 14);
  int table_rows = 0;
  if (uses_body_volumes) {
    for (const auto& volume : body_spheres)
      if (volume.used_for_capsule_table())
        table_rows += volume.span.isZero(0) ? 1 : 2;
  } else {
    table_rows = 2 * capsules.size();
  }
  check(table, table_rows);
  check(obstacles, pairs.size());
  check(self_collision, 0);
  check(inter_arm, 0);
  check(singularity, 1);
  for (double v : {singularity_length, singularity_minimum, boundary_activation, peer_max_age_us,
                   peer_max_skew_us, point_speed_limit, point_acceleration_limit})
    require(std::isfinite(v) && v > 0, "invalid collision/singularity setting");
  require(body_spheres.size() <= kMaxPoints, "body sphere capacity exceeded");
  require(std::isfinite(inter_arm_boundary_activation) && inter_arm_boundary_activation >= 0,
          "invalid inter-arm reference activation");
  require(!(self_collision.enabled || inter_arm.enabled) || !body_spheres.empty(),
          "collision requires body geometry");
  for (const auto& b : body_spheres)
    require(b.frame >= -1 && b.frame < kFrames && b.group >= 0 && b.group <= 10 &&
                b.offset.allFinite() && b.span.allFinite() && b.opening_axis.allFinite() &&
                (b.opening_axis.isZero(0) ||
                 (b.frame >= 0 && std::abs(b.opening_axis.norm() - 1) < 1e-8)) &&
                std::isfinite(b.radius) && b.radius > 0,
            "invalid body sphere");
  std::set<std::pair<int, int>> excluded;
  for (auto e : self_exclusions) {
    require(e.first >= 0 && e.second > e.first && e.second <= 10,
            "invalid self exclusion order/group");
    require(excluded.emplace(e.first, e.second).second, "duplicate self exclusion");
  }
  require(std::isfinite(recovery_braking_gain) && recovery_braking_gain > 0 &&
              recovery_projection_sweeps > 0 && recovery_projection_sweeps <= 12,
          "invalid recovery projection setting");
  require(position.margin < 1 && velocity.margin < 2, "joint margin removes admissible domain");
  require(rows <= max_constraints,
          "configured CBF capacity exceeded");
  require(2 * capsules.size() + points.size() <= kMaxPoints && obstacle_spheres.size() <= 32,
          "geometry capacity exceeded");
  require(!table.enabled || table_rows > 0,
          "table requires capsule geometry");
  require(!obstacles.enabled || !pairs.empty(), "obstacles require explicit collision pairs");
  const auto point = [&](int frame, const Eigen::Vector3d& v, double radius) {
    require(frame >= 0 && frame < kFrames && v.allFinite() && std::isfinite(radius) && radius > 0,
            "invalid point/frame/radius");
  };
  for (const auto& v : capsules) {
    point(v.frame, v.a, v.radius);
    point(v.frame, v.b, v.radius);
  }
  for (const auto& v : points)
    point(v.frame, v.offset, v.radius);
  for (const auto& v : obstacle_spheres)
    require(v.center.allFinite() && std::isfinite(v.radius) && v.radius > 0, "invalid obstacle");
  for (auto p : pairs)
    require(p.point >= 0 && p.point < (int)points.size() && p.obstacle >= 0 &&
                p.obstacle < (int)obstacle_spheres.size(),
            "invalid collision pair");
  require(world_from_base.matrix().allFinite() &&
              (world_from_base.linear().transpose() * world_from_base.linear())
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-9) &&
              std::abs(world_from_base.linear().determinant() - 1) < 1e-9,
          "invalid world transform");
}
Configuration load_configuration(const std::string& path, const std::string& controller_id) {
  require(!path.empty(), "enabled CBF requires a configuration file");
  std::set<std::string> inheritance_stack;
  const auto n = load_document(path, inheritance_stack);
  Configuration c;
  keys(n, {"robot", "execution", "objective", "qp", "solver", "verification",
           "constraints", "geometry", "diagnostics", "failure_policy"});
  auto r = n["robot"];
  keys(r, {"model", "joints", "gravity_command_convention"});
  choice(r, "model", "panda");
  require(get(r, "joints", 0) == 7, "CBF requires seven Panda joints");
  choice(r, "gravity_command_convention", "externally_gravity_compensated");
  auto e = n["execution"];
  keys(e, {"mode", "safety_torque_computation_method", "fallback_strategy", "max_constraints",
           "max_filter_time_us", "max_model_age_us", "max_callback_time_us",
           "analytic_nominal", "recovery_braking_gain_s_inv", "recovery_projection_sweeps"});
  choice(e, "mode", "synchronous");
  choice(e, "fallback_strategy", "bounded_recovery");
  const auto torque_method =
      get<std::string>(e, "safety_torque_computation_method", "projection");
  require(torque_method == "projection" || torque_method == "native_qp",
          "safety_torque_computation_method must be projection or native_qp");
  c.recovery_torque_method = torque_method == "native_qp" ? RecoveryTorqueMethod::native_qp
                                                          : RecoveryTorqueMethod::projection;
  c.analytic_nominal = boolean(e, "analytic_nominal", true);
  c.recovery_braking_gain = get(e, "recovery_braking_gain_s_inv", c.recovery_braking_gain);
  c.recovery_projection_sweeps = get(e, "recovery_projection_sweeps", c.recovery_projection_sweeps);
  c.max_constraints = get(e, "max_constraints", c.max_constraints);
  c.max_callback_time_us = get(e, "max_callback_time_us", c.max_callback_time_us);
  c.max_filter_time_us = get(e, "max_filter_time_us", c.max_filter_time_us);
  c.max_model_age_us = get(e, "max_model_age_us", c.max_model_age_us);
  auto o = n["objective"];
  keys(o, {"type", "joint_weights"});
  choice(o, "type", "torque_deviation");
  c.weights = vector<7>(o["joint_weights"]);
  auto q = n["qp"];
  keys(q, {"formulation"});
  choice(q, "formulation", "hard");
  auto so = n["solver"];
  keys(so, {"backend", "warm_start", "osqp", "qpoases", "piqp"});
  const auto backend = get(so, "backend", std::string("osqp"));
  require(backend == "osqp" || backend == "qpoases" || backend == "piqp", "unknown solver backend");
  c.backend = backend == "osqp"      ? Backend::osqp
              : backend == "qpoases" ? Backend::qpoases
                                     : Backend::piqp;
  c.warm_start = boolean(so, "warm_start", true);
  auto a = so["osqp"];
  keys(a, {"max_iterations", "absolute_tolerance", "relative_tolerance"});
  c.osqp_iterations = get(a, "max_iterations", c.osqp_iterations);
  c.osqp_abs = get(a, "absolute_tolerance", c.osqp_abs);
  c.osqp_rel = get(a, "relative_tolerance", c.osqp_rel);
  auto active = so["qpoases"];
  keys(active, {"max_working_set_recalculations", "termination_tolerance"});
  c.qpoases_recalculations =
      get(active, "max_working_set_recalculations", c.qpoases_recalculations);
  c.qpoases_tolerance = get(active, "termination_tolerance", c.qpoases_tolerance);
  auto interior = so["piqp"];
  keys(interior, {"max_iterations", "absolute_tolerance", "relative_tolerance"});
  c.piqp_iterations = get(interior, "max_iterations", c.piqp_iterations);
  c.piqp_abs = get(interior, "absolute_tolerance", c.piqp_abs);
  c.piqp_rel = get(interior, "relative_tolerance", c.piqp_rel);
  auto v = n["verification"];
  keys(v, {"hard_row_tolerance"});
  c.hard_tolerance = get(v, "hard_row_tolerance", c.hard_tolerance);
  auto co = n["constraints"];
  keys(co, {"joint_position", "joint_velocity", "joint_acceleration", "table", "obstacles",
            "self_collision", "inter_arm_collision", "singularity"});
  c.position.enabled = c.velocity.enabled = true;
  family(co["joint_position"], c.position, Family::position);
  family(co["joint_velocity"], c.velocity, Family::velocity);
  if (co["joint_position"] && co["joint_position"]["lower_limits_rad"])
    c.position_lower_limits = vector<7>(co["joint_position"]["lower_limits_rad"]);
  if (co["joint_position"] && co["joint_position"]["upper_limits_rad"])
    c.position_upper_limits = vector<7>(co["joint_position"]["upper_limits_rad"]);
  if (co["joint_velocity"] && co["joint_velocity"]["maximum_limits_rad_s"])
    c.velocity_limits = vector<7>(co["joint_velocity"]["maximum_limits_rad_s"]);
  family(co["joint_acceleration"], c.acceleration, Family::acceleration);
  family(co["table"], c.table, Family::table);
  family(co["obstacles"], c.obstacles, Family::obstacle);
  if (co["joint_acceleration"] && co["joint_acceleration"]["limits_rad_s2"])
    c.acceleration_limits = vector<7>(co["joint_acceleration"]["limits_rad_s2"]);
  family(co["self_collision"], c.self_collision, Family::self_collision);
  c.boundary_activation = get(co["self_collision"], "boundary_activation_m", c.boundary_activation);
  family(co["inter_arm_collision"], c.inter_arm, Family::inter_arm);
  family(co["singularity"], c.singularity, Family::singularity);
  auto inter = co["inter_arm_collision"], singular = co["singularity"];
  c.peer_max_age_us = get(inter, "max_peer_age_us", c.peer_max_age_us);
  const auto inter_policy = get<std::string>(inter, "policy", "independent");
  require(inter_policy == "independent", "only independent inter-arm policy is supported");
  c.peer_max_skew_us = get(inter, "max_peer_skew_us", c.peer_max_skew_us);
  c.point_speed_limit = get(inter, "point_speed_limit_m_s", c.point_speed_limit);
  c.point_acceleration_limit =
      get(inter, "point_acceleration_limit_m_s2", c.point_acceleration_limit);
  c.singularity_length = get(singular, "characteristic_length_m", c.singularity_length);
  c.singularity_minimum = get(singular, "minimum_normalized_sigma", c.singularity_minimum);
  c.inter_arm_boundary_activation = get(inter, "reference_activation_m", 0.);
  if (inter && inter["reference_activation_m"])
    require(c.inter_arm_boundary_activation > 0, "inter-arm reference activation must be positive");
  auto ge = n["geometry"];
  keys(ge, {"file", "files_by_controller"});
  require(ge && (bool(ge["file"]) != bool(ge["files_by_controller"])),
          "geometry requires exactly one of file or files_by_controller");
  std::string geometry_file;
  if (ge["files_by_controller"]) {
    const auto files = ge["files_by_controller"];
    keys(files, {"left", "right"});
    require(files["left"] && files["right"], "dual geometry requires left and right files");
    require(controller_id == "left" || controller_id == "right",
            "dual geometry requires controller selector left/right");
    geometry_file = files[controller_id].as<std::string>();
  } else {
    geometry_file = ge["file"].as<std::string>();
  }
  require(!geometry_file.empty(), "geometry file required");
  std::filesystem::path geometry_path = geometry_file;
  if (geometry_path.is_relative())
    geometry_path = std::filesystem::path(path).parent_path() / geometry_path;
  geometry(geometry_path, c);
  auto d = n["diagnostics"];
  keys(d, {"enabled", "record_constraint_residuals", "record_computation_time",
           });
  const bool diagnostics = boolean(d, "enabled", true);
  c.diagnostic_residuals = boolean(d, "record_constraint_residuals", true) && diagnostics;
  c.diagnostic_times = boolean(d, "record_computation_time", true) && diagnostics;
  auto f = n["failure_policy"];
  keys(f, {"solver_failure", "infeasible_qp", "invalid_model", "deadline_overrun"});
  for (auto key : {"solver_failure", "infeasible_qp", "deadline_overrun"})
    choice(f, key, "request_existing_protective_handling");
  choice(f, "invalid_model", "terminate_control");
  c.validate();
  return c;
}
const char* name(Backend b) noexcept {
  return b == Backend::osqp ? "osqp" : b == Backend::qpoases ? "qpoases" : "piqp";
}
const char* name(Status s) noexcept {
  switch (s) {
    case Status::disabled:
      return "disabled";
    case Status::solved_hard:
      return "solved_hard";
    case Status::infeasible:
      return "infeasible";
    case Status::iteration_limit:
      return "iteration_limit";
    case Status::numerical_failure:
      return "numerical_failure";
    case Status::invalid_model:
      return "invalid_model";
    case Status::invalid_domain:
      return "invalid_domain";
    case Status::constraint_violation:
      return "constraint_violation";
    case Status::capacity_exceeded:
      return "capacity_exceeded";
    case Status::timing_overrun:
      return "timing_overrun";
    case Status::stale_peer:
      return "stale_peer";
    case Status::verified_recovery:
      return "verified_recovery";
  }
  return "unknown";
}

}  // namespace cbf_safety_filter
