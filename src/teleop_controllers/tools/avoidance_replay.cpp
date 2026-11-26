// Read-only offline validation. No Robot, ROS nodes or command publication.
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "cbf_safety_filter/kinematics.hpp"
#include "cbf_safety_filter/safety_filter.hpp"
#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/urdf_model.hpp"

namespace ft = teleop_controllers;
namespace cbf = cbf_safety_filter;
using Clock = std::chrono::steady_clock;
uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
}
double smooth_value(const ft::RobotModelState& m, double length) {
  cbf::Jacobian j = m.frame_jacobian[8];
  j.topRows<3>() /= length;
  const auto singular = Eigen::JacobiSVD<cbf::Jacobian>(j).singularValues();
  return 1. / singular.array().square().inverse().sum();
}
template <class V>
void json_vector(const V& value) {
  std::cout << '[';
  for (int j = 0; j < value.size(); ++j)
    std::cout << (j ? "," : "") << value[j];
  std::cout << ']';
}

int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("usage: avoidance_replay CBF_YAML URDF INPUT_STREAM");
    const ft::UrdfPandaModel dynamics(argv[2]);
    auto config = cbf::load_configuration(argv[1]);
    if (config.inter_arm.enabled)
      throw std::runtime_error("single-arm replay has no peer trajectory");
    // Profiles are explicit offline overrides; production YAMLs are never written.
    std::array<cbf::Configuration, 4> profiles{config, config, config, config};
    for (int i : {1, 2}) {
      auto& c = profiles[i];
      c.position.enabled = c.velocity.enabled = c.acceleration.enabled = false;
      c.obstacles.enabled = c.self_collision.enabled = c.inter_arm.enabled = false;
      c.table.enabled = i == 1;
      c.singularity.enabled = i == 2;
    }
    profiles[3].table.enabled = profiles[3].singularity.enabled = true;
    const std::array<const char*, 4> names{"configured", "table_only", "singularity_only",
                                           "combined"};
    std::array<std::unique_ptr<cbf::SafetyFilter>, 4> filters;
    std::array<std::unique_ptr<cbf::ConstraintBuilder>, 4> builders;
    for (int i = 0; i < 4; ++i) {
      profiles[i].validate();
      filters[i] = std::make_unique<cbf::SafetyFilter>(profiles[i]);
      builders[i] = std::make_unique<cbf::ConstraintBuilder>(profiles[i]);
    }
    std::ifstream input(argv[3], std::ios::binary);
    char magic[8];
    uint64_t count = 0, fields = 0;
    input.read(magic, 8);
    input.read(reinterpret_cast<char*>(&count), 8);
    input.read(reinterpret_cast<char*>(&fields), 8);
    if (!input || std::memcmp(magic, "AVOID01\n", 8) || fields != 54)
      throw std::runtime_error("invalid replay stream");
    std::cout << std::setprecision(17) << std::boolalpha;
    double max_gradient_error = 0, max_bias_error = 0, max_point_bias_error = 0;
    uint64_t previous_cycle = 0;
    for (uint64_t sample = 0; sample < count; ++sample) {
      std::array<double, 54> row{};
      input.read(reinterpret_cast<char*>(row.data()), sizeof(row));
      if (!input || !Eigen::Map<const Eigen::Matrix<double, 54, 1>>(row.data()).allFinite())
        throw std::runtime_error("truncated/nonfinite replay stream");
      const uint64_t cycle = static_cast<uint64_t>(row[0]);
      if (cycle <= previous_cycle)
        throw std::runtime_error("duplicate or unordered control cycle");
      previous_cycle = cycle;
      const double dt = row[1];
      const ft::Vector7 q = Eigen::Map<const ft::Vector7>(row.data() + 2);
      const ft::Vector7 dq = Eigen::Map<const ft::Vector7>(row.data() + 9);
      const ft::Vector7 previous = Eigen::Map<const ft::Vector7>(row.data() + 16);
      const ft::Vector7 nominal = Eigen::Map<const ft::Vector7>(row.data() + 23);
      const ft::Vector7 issued = Eigen::Map<const ft::Vector7>(row.data() + 30);
      const ft::Vector7 caps = Eigen::Map<const ft::Vector7>(row.data() + 37);
      const double rate = row[44];
      const Eigen::Vector3d position = Eigen::Map<const Eigen::Vector3d>(row.data() + 45);
      Eigen::Quaterniond orientation;
      orientation.coeffs() = Eigen::Map<const Eigen::Vector4d>(row.data() + 48);
      ft::UrdfModelOutput output;
      if (!dynamics.evaluate(q, dq, dynamics.standard_flange_to_ee(), output))
        throw std::runtime_error("invalid URDF evaluation");
      const auto& m = output.state;
      const double pose_error = std::max(
          (position - m.frame_pose[8].translation()).cwiseAbs().maxCoeff(),
          (orientation.toRotationMatrix() - m.frame_pose[8].linear()).cwiseAbs().maxCoeff());
      if (pose_error > 1e-6)
        throw std::runtime_error(
            "recorded tool frame differs: supply matching URDF/tool convention");
      const auto bounds = ft::actuator_bounds(previous, caps, rate, dt);
      if (!bounds)
        throw std::runtime_error("invalid recorded actuator interval");
      const auto bounded = bounds->clamp(nominal);
      const cbf::TorqueBounds box{bounds->lower, bounds->upper};
      const cbf::ModelView view{m.q,     m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian,
                                now_ns()};
      const auto singular = cbf::singularity_kinematics(view, config.singularity_length);
      cbf::Jacobian normalized = m.frame_jacobian[8];
      normalized.topRows<3>() /= config.singularity_length;
      const double sigma = Eigen::JacobiSVD<cbf::Jacobian>(normalized).singularValues()[5];
      double hmin = 1e100, psimin = 1e100, hdmin = 0;
      int critical = -1;
      cbf::PointDefinition bias_point;
      const auto check_endpoint = [&](const cbf::PointDefinition& endpoint, size_t index) {
        const auto p = cbf::point_kinematics(view, endpoint, config.world_from_base);
        const double h =
            p.position.z() - endpoint.radius - config.table.height - config.table.clearance;
        if (h < hmin) {
          hmin = h;
          hdmin = p.velocity.z();
          critical = static_cast<int>(index);
          bias_point = endpoint;
        }
        psimin = std::min(psimin, p.velocity.z() + config.table.alpha1 * h);
      };
      if (config.uses_body_volumes) {
        for (size_t i = 0; i < config.body_spheres.size(); ++i) {
          const auto& body = config.body_spheres[i];
          if (!body.used_for_capsule_table())
            continue;
          const auto volume = cbf::position_body_volume(body, view, config);
          check_endpoint(volume, i);
          if (!volume.span.isZero(0))
            check_endpoint({volume.frame, volume.offset + volume.span, volume.radius}, i);
        }
      } else {
        for (size_t i = 0; i < config.capsules.size(); ++i) {
          const auto& capsule = config.capsules[i];
          for (const auto& offset : {capsule.a, capsule.b})
            check_endpoint({capsule.frame, offset, capsule.radius}, i);
        }
        if (!config.capsules.empty()) {
          const auto& capsule = config.capsules.back();
          bias_point = {capsule.frame, capsule.a, capsule.radius};
        }
      }
      // Independent central differences of FK/SVD at representative raw samples.
      if (sample % 1000 == 0 && singular.valid) {
        const double eps = 1e-5;
        for (int j = 0; j < 7; ++j) {
          auto plus = q, minus = q;
          plus[j] += eps;
          minus[j] -= eps;
          ft::UrdfModelOutput p, n;
          if (!dynamics.evaluate(plus, dq, dynamics.standard_flange_to_ee(), p) ||
              !dynamics.evaluate(minus, dq, dynamics.standard_flange_to_ee(), n))
            throw std::runtime_error("finite difference model failure");
          const double fd = (smooth_value(p.state, config.singularity_length) -
                             smooth_value(n.state, config.singularity_length)) /
                            (2 * eps);
          max_gradient_error = std::max(max_gradient_error, std::abs(fd - singular.gradient[j]));
        }
        const double t = .001;
        ft::UrdfModelOutput p, n;
        dynamics.evaluate(q + t * dq, dq, dynamics.standard_flange_to_ee(), p);
        dynamics.evaluate(q - t * dq, dq, dynamics.standard_flange_to_ee(), n);
        const double fd = (smooth_value(p.state, config.singularity_length) -
                           2 * smooth_value(m, config.singularity_length) +
                           smooth_value(n.state, config.singularity_length)) /
                          (t * t);
        max_bias_error = std::max(max_bias_error, std::abs(fd - singular.bias));
        if (critical >= 0) {
          const auto& point = bias_point;
          const auto actual = cbf::point_kinematics(view, point, config.world_from_base);
          const Eigen::Vector3d fp =
              config.world_from_base * (p.state.frame_pose[point.frame] * point.offset);
          const Eigen::Vector3d fn =
              config.world_from_base * (n.state.frame_pose[point.frame] * point.offset);
          max_point_bias_error =
              std::max(max_point_bias_error,
                       ((fp - 2 * actual.position + fn) / (t * t) - actual.acceleration_bias).norm());
        }
      }
      std::cout << "{\"cycle\":" << cycle << ",\"active\":" << (row[53] != 0)
                << ",\"state_stamp_ns\":" << static_cast<uint64_t>(row[52])
                << ",\"table_h_min_m\":" << hmin << ",\"table_psi_min_m_s\":" << psimin
                << ",\"critical_capsule\":" << critical << ",\"critical_height_rate_m_s\":" << hdmin
                << ",\"sigma_min\":" << sigma << ",\"singularity_valid\":" << singular.valid
                << ",\"singularity_smooth_sigma\":" << std::sqrt(std::max(0., singular.value))
                << ",\"singularity_h\":"
                << singular.value - config.singularity_minimum * config.singularity_minimum
                << ",\"singularity_psi\":"
                << singular.gradient.dot(dq) +
                       config.singularity.alpha1 * (singular.value - config.singularity_minimum *
                                                                         config.singularity_minimum)
                << ",\"pose_error\":" << pose_error << ",\"recorded_nominal_clip_difference\":"
                << (bounded - issued).cwiseAbs().maxCoeff() << ",\"profiles\":{";
      for (int variant = 0; variant < 4; ++variant) {
        const uint64_t now = now_ns();
        const cbf::ModelView fresh{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian,
                                   now};
        const auto& rows = builders[variant]->build(fresh, dt, now, &box);
        double violation = -1e100;
        int impossible = 0;
        for (int k = 0; k < rows.count; ++k) {
          const auto& r = rows.rows[k];
          violation = std::max(violation, r.g.dot(bounded) - r.bound);
          const double minimum =
              (r.g.array() * (r.g.array() >= 0).select(box.lower.array(), box.upper.array()))
                  .sum() -
              r.bound;
          impossible += minimum > profiles[variant].hard_tolerance;
        }
        const uint64_t solve_now = now_ns();
        const cbf::ModelView solve_view{
            m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, solve_now};
        const auto& result = filters[variant]->compute(solve_view, nominal, box, dt, solve_now);
        const ft::Vector7 outgoing = bounds->clamp(result.torque);
        const bool verified =
            result.accepted() &&
            (outgoing - result.torque).cwiseAbs().maxCoeff() <= profiles[variant].hard_tolerance &&
            filters[variant]->verify_command(outgoing);
        std::cout << (variant ? "," : "") << '"' << names[variant] << "\":{\"domain_status\":\""
                  << cbf::name(rows.status) << "\",\"rows\":" << rows.count
                  << ",\"nominal_max_row_violation\":" << violation
                  << ",\"impossible_rows\":" << impossible << ",\"status\":\""
                  << cbf::name(result.status) << "\",\"verified\":" << verified
                  << ",\"correction_norm\":" << (verified ? (outgoing - bounded).norm() : 0.)
                  << ",\"filter_us\":" << result.total_us << ",\"filtered_torque\":";
        json_vector(outgoing);
        std::cout << '}';
      }
      std::cout << "}}\n";
    }
    if (input.peek() != EOF)
      throw std::runtime_error("trailing replay data");
    std::cerr << std::setprecision(17) << "{\"samples\":" << count
              << ",\"max_singularity_gradient_fd_error\":" << max_gradient_error
              << ",\"max_singularity_bias_fd_error\":" << max_bias_error
              << ",\"max_table_point_bias_fd_error\":" << max_point_bias_error << "}\n";
    if (max_gradient_error > 1e-6 || max_bias_error > 1e-6 || max_point_bias_error > 1e-6)
      return 3;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
