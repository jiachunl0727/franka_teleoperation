// Offline synthetic boundary challenges seeded from a real recorded pose.
// These are explicitly not recorded trajectories or physical stopping tests.
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>

#include "cbf_safety_filter/kinematics.hpp"
#include "cbf_safety_filter/safety_filter.hpp"
#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/urdf_model.hpp"

namespace ft = teleop_controllers;
namespace cbf = cbf_safety_filter;
uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("usage: avoidance_boundary_probe CBF_YAML URDF STREAM");
    const ft::UrdfPandaModel model(argv[2]);
    const auto base = cbf::load_configuration(argv[1]);
    std::ifstream input(argv[3], std::ios::binary);
    char magic[8];
    uint64_t count = 0, fields = 0;
    input.read(magic, 8);
    input.read(reinterpret_cast<char*>(&count), 8);
    input.read(reinterpret_cast<char*>(&fields), 8);
    std::array<double, 54> data{};
    input.read(reinterpret_cast<char*>(data.data()), sizeof(data));
    if (!input || std::memcmp(magic, "AVOID01\n", 8) || !count || fields != 54)
      throw std::runtime_error("invalid stream");
    const ft::Vector7 seed = Eigen::Map<const ft::Vector7>(data.data() + 2);
    const ft::Vector7 caps = Eigen::Map<const ft::Vector7>(data.data() + 37);
    const double rate = data[44];
    std::mt19937 generator(20261109);
    const ft::Vector7 lower =
        (ft::Vector7() << -2.7973, -1.6628, -2.7973, -2.9718, -2.7973, .0825, -2.7973).finished();
    const ft::Vector7 upper =
        (ft::Vector7() << 2.7973, 1.6628, 2.7973, -.1698, 2.7973, 3.6525, 2.7973).finished();
    std::cout << std::setprecision(17) << std::boolalpha;
    for (int family = 0; family < 2; ++family) {
      auto config = base;
        config.position.enabled = config.velocity.enabled = config.acceleration.enabled = false;
      config.obstacles.enabled = config.self_collision.enabled = config.inter_arm.enabled = false;
      config.table.enabled = family == 0;
      config.singularity.enabled = family == 1;
      config.validate();
      const auto margin = [&](const ft::Vector7& q) {
        ft::UrdfModelOutput o;
        if (!model.evaluate(q, ft::Vector7::Zero(), model.standard_flange_to_ee(), o))
          throw std::runtime_error("model failure");
        const auto& m = o.state;
        const cbf::ModelView view{m.q,     m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian,
                                  now_ns()};
        if (family) {
          const auto s = cbf::singularity_kinematics(view, config.singularity_length);
          return s.valid ? s.value - config.singularity_minimum * config.singularity_minimum : -1.;
        }
        double h = 1e100;
        if (config.uses_body_volumes) {
          const auto snapshot = cbf::collision_snapshot(view, config);
          for (int i = 0; i < snapshot.count; ++i) {
            if (!config.body_spheres[i].used_for_capsule_table())
              continue;
            const auto& volume = snapshot.spheres[i];
            const double bottom = volume.position.z() + std::min(0., volume.span.z()) - volume.radius;
            h = std::min(h, bottom - config.table.height - config.table.clearance);
          }
        } else {
          for (const auto& capsule : config.capsules)
            for (const auto& offset : {capsule.a, capsule.b}) {
              const auto p = cbf::point_kinematics(view, {capsule.frame, offset, capsule.radius},
                                                   config.world_from_base);
              h = std::min(
                  h, p.position.z() - capsule.radius - config.table.height - config.table.clearance);
            }
        }
        return h;
      };
      const double target = family ? 1e-7 : 1e-4;
      ft::Vector7 safe = seed, unsafe = seed;
      if (margin(safe) <= target)
        throw std::runtime_error("recorded seed is outside challenge's safe domain");
      bool found = false;
      for (int sample = 0; sample < 20000 && !found; ++sample) {
        for (int j = 0; j < 7; ++j)
          unsafe[j] = std::uniform_real_distribution<double>(lower[j], upper[j])(generator);
        found = margin(unsafe) < 0;
      }
      if (!found)
        throw std::runtime_error("no deterministic boundary bracket found");
      for (int i = 0; i < 70; ++i) {
        const ft::Vector7 middle = (safe + unsafe) / 2;
        if (margin(middle) > target)
          safe = middle;
        else
          unsafe = middle;
      }
      ft::UrdfModelOutput o;
      model.evaluate(safe, ft::Vector7::Zero(), model.standard_flange_to_ee(), o);
      const auto& m = o.state;
      // Synthetic rest/holding history: magnitude caps and 350 Nm/s slew are
      // retained, while previous torque is c=0 instead of the bag's moving history.
      const auto bounds = ft::actuator_bounds(m.coriolis, caps, rate, .001);
      if (!bounds)
        throw std::runtime_error("challenge actuator bounds invalid");
      const cbf::TorqueBounds box{bounds->lower, bounds->upper};
      const uint64_t now = now_ns();
      const cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, now};
      auto builder = std::make_unique<cbf::ConstraintBuilder>(config);
      const auto& rows = builder->build(view, .001, now, &box);
      if (rows.status != cbf::Status::solved_hard || !rows.count)
        throw std::runtime_error("challenge row not active");
      double worst = -1e100;
      ft::Vector7 adversarial = m.coriolis;
      for (int i = 0; i < rows.count; ++i) {
        const auto& row = rows.rows[i];
        const ft::Vector7 command =
            (row.g.array() >= 0).select(box.upper.array(), box.lower.array());
        const double violation = row.g.dot(command) - row.bound;
        if (violation > worst) {
          worst = violation;
          adversarial = command;
        }
      }
      auto filter = std::make_unique<cbf::SafetyFilter>(config);
      const uint64_t solve_now = now_ns();
      const cbf::ModelView solve_view{
          m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, solve_now};
      const auto& result = filter->compute(solve_view, adversarial, box, .001, solve_now);
      const ft::Vector7 outgoing = bounds->clamp(result.torque);
      const bool verified = result.accepted() && filter->verify_command(outgoing);
      double residual = -1e100;
      for (int i = 0; i < rows.count; ++i)
        residual = std::max(residual, rows.rows[i].g.dot(outgoing) - rows.rows[i].bound);
      std::cout << "{\"family\":\"" << (family ? "singularity" : "table")
                << "\",\"synthetic\":true,\"seed_cycle\":" << data[0]
                << ",\"boundary_margin\":" << margin(safe) << ",\"nominal_row_violation\":" << worst
                << ",\"rows\":" << rows.count << ",\"status\":\"" << cbf::name(result.status)
                << "\",\"verified\":" << verified
                << ",\"correction_norm\":" << (outgoing - adversarial).norm()
                << ",\"outgoing_max_row_residual\":" << residual
                << ",\"filter_us\":" << result.total_us << "}\n";
      if (worst <= 1e-6 || !verified || residual > config.hard_tolerance ||
          (outgoing - adversarial).norm() < 1e-6)
        return 3;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
