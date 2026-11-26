// Read-only reference-path replay. Probe law deliberately does not predict
// torque or physical motion. Fresh recorded states are exogenous, not a rollout.
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "cbf_safety_filter/safety_filter.hpp"
#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/controller_manager.hpp"
#include "teleop_controllers/urdf_model.hpp"

namespace ft = teleop_controllers;
namespace cbf = cbf_safety_filter;
struct ProbeLaw : ft::TorqueController {
  ft::NominalTorque compute(const ft::ArmState&, const ft::ControlReference&,
                            double) noexcept override {
    return {ft::Vector7::Zero(), ft::StateFault::none};
  }
  void reset(const ft::ArmState&) noexcept override {}
};
ft::Vector6 difference(const ft::CartesianReference& a, const ft::CartesianReference& b) {
  ft::Vector6 out;
  out.head<3>() = a.position - b.position;
  Eigen::Quaterniond q = a.orientation * b.orientation.conjugate();
  if (q.w() < 0)
    q.coeffs() *= -1;
  const Eigen::AngleAxisd angle(q.normalized());
  out.tail<3>() = angle.axis() * angle.angle();
  return out;
}
int main(int argc, char** argv) {
  try {
    if (argc != 5)
      throw std::runtime_error("usage: reference_projection_replay CBF URDF STATES REFERENCES");
    const auto config = cbf::load_configuration(argv[1]);
    ft::UrdfPandaModel model(argv[2]);
    std::cout << std::boolalpha << std::setprecision(17);
    for (bool shaping : {false, true}) {
      std::ifstream states(argv[3], std::ios::binary), references(argv[4], std::ios::binary);
      char magic[8];
      uint64_t count = 0, fields = 0;
      states.read(magic, 8);
      states.read(reinterpret_cast<char*>(&count), 8);
      states.read(reinterpret_cast<char*>(&fields), 8);
      if (!states || std::memcmp(magic, "AVOID01\n", 8) || fields != 54 || !count)
        throw std::runtime_error("invalid state stream");
      // Match the runtime's distinct law identities: the manager uses law
      // transitions to seed its measured-pose reference on engagement.
      ProbeLaw teleoperation, orientation, holding, damping;
      ft::ManagerParameters p;
      p.reference_shaping_enabled = shaping;
      ft::ControllerManager manager(p, ft::HomingParameters{}, teleoperation, orientation, holding,
                                    damping);
      cbf::SafetyFilter filter(config);
      manager.reference_projection(
          [](const void* context, const ft::Vector6& step) noexcept -> ft::Vector6 {
            return static_cast<const cbf::SafetyFilter*>(context)->reference_normals().project(
                step);
          },
          &filter);
      ft::CartesianReference previous, raw_previous;
      bool previously_active = false;
      uint64_t active = 0, near = 0, blocked = 0, unsafe_steps = 0, unsafe_twists = 0;
      double max_step_violation = 0, max_twist_violation = 0;
      Eigen::Vector3d final_reference = Eigen::Vector3d::Zero();
      for (uint64_t sample = 0; sample < count; ++sample) {
        std::array<double, 54> row{};
        std::array<double, 13> request{};
        states.read(reinterpret_cast<char*>(row.data()), sizeof(row));
        references.read(reinterpret_cast<char*>(request.data()), sizeof(request));
        if (!states || !references)
          throw std::runtime_error("truncated replay input");
        ft::UrdfModelOutput evaluated;
        if (!model.evaluate(Eigen::Map<const ft::Vector7>(row.data() + 2),
                            Eigen::Map<const ft::Vector7>(row.data() + 9),
                            model.standard_flange_to_ee(), evaluated))
          throw std::runtime_error("invalid model");
        const auto& m = evaluated.state;
        ft::ArmState state;
        state.q = m.q;
        state.dq = m.dq;
        state.mass = m.mass;
        state.coriolis = m.coriolis;
        state.jacobian = m.frame_jacobian[8];
        state.position = m.frame_pose[8].translation();
        state.orientation = m.frame_pose[8].linear();
        state.previous_torque = Eigen::Map<const ft::Vector7>(row.data() + 16);
        const auto bounds = ft::actuator_bounds(
            state.previous_torque, Eigen::Map<const ft::Vector7>(row.data() + 37), row[44], row[1]);
        if (!bounds)
          throw std::runtime_error("invalid interval");
        const cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, 1};
        if (filter.prepare(view, {bounds->lower, bounds->upper}, row[1], 1) !=
            cbf::Status::solved_hard)
          throw std::runtime_error("recorded state outside barrier domain");
        ft::Command command;
        std::strcpy(command.session.data(), "offline-reference-replay");
        command.sequence = static_cast<uint64_t>(row[0]);
        command.receipt_ns = command.source_ns = static_cast<uint64_t>(row[52]);
        command.valid = true;
        command.enabled = row[53] != 0;
        command.released = !command.enabled;
        command.position = Eigen::Map<const Eigen::Vector3d>(request.data());
        command.orientation.coeffs() = Eigen::Map<const Eigen::Vector4d>(request.data() + 3);
        command.twist = Eigen::Map<const ft::Vector6>(request.data() + 7);
        const ft::CartesianReference raw{command.position, command.orientation, command.twist};
        if (command.enabled && !previously_active) {
          previous = raw_previous = {state.position, state.orientation, ft::Vector6::Zero()};
        }
        manager.accept(command, command.receipt_ns);
        const auto output = manager.update(state, command.receipt_ns, row[1]).control;
        if (command.enabled && !output.motion_enabled)
          throw std::runtime_error("replay input unexpectedly disarmed");
        ft::CartesianReference applied{output.target_position, output.target_orientation,
                                       output.desired_twist};
        // A source anchored at the measured pose must not jump when armed,
        // including when a virtual reference normal is active from startup.
        const ft::CartesianReference measured{state.position, state.orientation,
                                              ft::Vector6::Zero()};
        if (command.enabled && !previously_active && difference(raw, measured).norm() < 1e-6 &&
            difference(applied, measured).norm() > 1e-5)
          throw std::runtime_error("reference replay jumped at an already anchored first input");
        if (command.enabled) {
          ++active;
          const auto& normals = filter.reference_normals();
          near += normals.count > 0;
          const auto raw_step = difference(raw, raw_previous), step = difference(applied, previous);
          bool rejected = false;
          double step_violation = 0, twist_violation = 0;
          for (int i = 0; i < normals.count; ++i) {
            const auto& n = normals.normals[i];
            rejected |= n.dot(raw_step) < -1e-10;
            step_violation = std::max(step_violation, -n.dot(step));
            twist_violation = std::max(twist_violation, -n.dot(applied.twist));
          }
          blocked += rejected;
          unsafe_steps += step_violation > 1e-9;
          unsafe_twists += twist_violation > 1e-9;
          max_step_violation = std::max(max_step_violation, step_violation);
          max_twist_violation = std::max(max_twist_violation, twist_violation);
          final_reference = applied.position;
        }
        previously_active = command.enabled;
        previous = applied;
        raw_previous = raw;
      }
      if (states.peek() != EOF || references.peek() != EOF)
        throw std::runtime_error("trailing replay data");
      std::cout << "{\"scope\":\"frozen reference-only replay; no torque law or physical rollout\","
                << "\"shaping\":" << shaping << ",\"samples\":" << count << ",\"active\":" << active
                << ",\"near_boundary_active\":" << near << ",\"inward_inputs\":" << blocked
                << ",\"unsafe_reference_steps\":" << unsafe_steps
                << ",\"unsafe_reference_twists\":" << unsafe_twists
                << ",\"maximum_step_violation\":" << max_step_violation
                << ",\"maximum_twist_violation\":" << max_twist_violation
                << ",\"final_reference_xyz\":[" << final_reference.x() << ',' << final_reference.y()
                << ',' << final_reference.z() << "]}\n";
      if (unsafe_steps || unsafe_twists)
        return 3;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
