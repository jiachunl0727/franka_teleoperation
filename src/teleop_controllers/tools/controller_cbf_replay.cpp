// Offline paired-model replay only. Never opens a robot or publishes commands.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "cbf_safety_filter/safety_filter.hpp"
#include "cbf_safety_filter/problem_snapshot.hpp"
#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/inverse_dynamics_controller.hpp"
#include "teleop_controllers/model_snapshot.hpp"

namespace ft = teleop_controllers;
namespace cbf = cbf_safety_filter;
using Clock = std::chrono::steady_clock;
template <class Derived>
void read(std::istream& input, Eigen::MatrixBase<Derived>& values) {
  for (int i = 0; i < values.size(); ++i)
    input >> values.derived().data()[i];
}
template <class Derived>
void json_vector(const Eigen::MatrixBase<Derived>& values) {
  std::cout << '[';
  for (int i = 0; i < values.size(); ++i)
    std::cout << (i ? "," : "") << values[i];
  std::cout << ']';
}

int main(int argc, char** argv) {
  try {
    if (argc != 3)
      throw std::runtime_error("usage: controller_cbf_replay CBF_YAML REPLAY_INPUT");
    auto config = cbf::load_configuration(argv[1]);
    // Historical async captures are solved as frozen synchronous problems.
    std::ifstream input(argv[2]);
    std::string version;
    input >> version;
    if (version != "CONTROLLER_REPLAY_V1" && version != "CONTROLLER_REPLAY_V2")
      throw std::runtime_error("invalid replay input version");
    ft::InverseDynamicsParameters parameters;
    read(input, parameters.stiffness);
    input >> parameters.damping_ratio;
    read(input, parameters.nullspace_stiffness);
    read(input, parameters.nullspace_damping);
    read(input, parameters.common.home_joint_position);
    if (version == "CONTROLLER_REPLAY_V1") {
      bool posture_active = false;
      input >> posture_active;
      if (!posture_active)
        throw std::runtime_error("legacy replay disables posture control; regenerate for always-active posture");
    }
    input >> parameters.common.cartesian_wrench_limits_enabled >>
        parameters.common.cartesian_force_limit >> parameters.common.cartesian_moment_limit;
    if (!input || !parameters.valid())
      throw std::runtime_error("invalid replay parameters");
    ft::InverseDynamicsController law(parameters);
    auto original = std::make_unique<cbf::SafetyFilter>(config);
    auto replacement = std::make_unique<cbf::SafetyFilter>(config);
    std::cout << std::setprecision(17) << std::boolalpha;
    std::string model_path, problem_path;
    uint64_t cycle;
    while (input >> std::quoted(model_path) >> std::quoted(problem_path) >> cycle) {
      ft::ArmState state;
      ft::CartesianReference reference;
      read(input, state.position);
      read(input, state.orientation.coeffs());
      read(input, reference.position);
      read(input, reference.orientation.coeffs());
      read(input, reference.twist);
      ft::Vector7 cap;
      double rate;
      read(input, cap);
      input >> rate;
      if (!input)
        throw std::runtime_error("truncated replay reference");
      ft::ModelSnapshot snapshot;
      std::ifstream model_file(model_path), problem_file(problem_path);
      auto captured = std::make_unique<cbf::ProblemSnapshotV2>();
      if (!ft::read_model_snapshot(model_file, snapshot) ||
          !cbf::read_problem_snapshot(problem_file, *captured) || snapshot.cycle != cycle ||
          captured->cycle != cycle)
        throw std::runtime_error("model/problem/reference cycle mismatch");
      const auto& m = snapshot.state;
      state.q = m.q;
      state.dq = m.dq;
      state.mass = m.mass;
      state.coriolis = m.coriolis;
      state.previous_torque = m.previous_torque;
      state.jacobian = m.frame_jacobian[8];
      const auto bounds = ft::actuator_bounds(m.previous_torque, cap, rate, snapshot.period);
      if (!bounds || !bounds->lower.isApprox(captured->bounds.lower, 1e-12) ||
          !bounds->upper.isApprox(captured->bounds.upper, 1e-12))
        throw std::runtime_error("recorded actuator bounds do not reconstruct exactly");
      if (!snapshot.nominal.isApprox(captured->nominal, 1e-12))
        throw std::runtime_error("recorded nominal mismatch");
      const auto start = Clock::now();
      const auto nominal = law.compute(state, reference, snapshot.period);
      const double controller_us =
          std::chrono::duration<double, std::micro>(Clock::now() - start).count();
      if (nominal.fault != ft::StateFault::none)
        throw std::runtime_error(ft::state_fault_name(nominal.fault));
      for (int variant = 0; variant < 2; ++variant) {
        auto& filter = variant ? *replacement : *original;
        const ft::Vector7 torque = variant ? nominal.torque : snapshot.nominal;
        const auto now =
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count();
        const cbf::ModelView view{m.q,
                                  m.dq,
                                  m.mass,
                                  m.coriolis,
                                  m.frame_pose,
                                  m.frame_jacobian,
                                  static_cast<uint64_t>(now)};
        const auto& result = filter.compute(view, torque, captured->bounds, snapshot.period, now);
        const auto inspection = filter.inspect_command(bounds->clamp(torque));
        const ft::Vector7 outgoing = bounds->clamp(result.torque);
        const double clipping = (outgoing - result.torque).cwiseAbs().maxCoeff();
        const bool verified = result.accepted() && clipping <= config.hard_tolerance &&
                              filter.verify_command(outgoing);
        std::cout << "{\"cycle\":" << cycle << ",\"law\":\""
                  << (variant ? "inverse_dynamics" : "recorded_impedance") << "\",\"status\":\""
                  << cbf::name(result.status) << "\",\"accepted\":" << result.accepted()
                  << ",\"outgoing_verified\":" << verified
                  << ",\"bounded_nominal_feasible\":" << inspection.feasible()
                  << ",\"box_impossible_rows\":" << inspection.impossible_rows
                  << ",\"max_nominal_row_violation\":" << inspection.max_row_violation
                  << ",\"rows\":" << result.rows << ",\"filter_us\":" << result.total_us
                  << ",\"controller_us\":" << (variant ? controller_us : 0.)
                  << ",\"nominal_torque\":";
        json_vector(torque);
        std::cout << ",\"bounded_nominal_torque\":";
        json_vector(bounds->clamp(torque));
        std::cout << ",\"filtered_torque\":";
        json_vector(outgoing);
        std::cout << "}\n";
      }
    }
    if (!input.eof())
      throw std::runtime_error("malformed replay input");
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
