#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cbf_safety_filter/problem_snapshot.hpp"
#include "cbf_safety_filter/qp_solver.hpp"

namespace cbf = cbf_safety_filter;
int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "Usage: cbf_replay CONFIG.yaml PROBLEM.qp.txt [--extended]\n";
    return 2;
  }
  try {
    if (argc == 4 && std::string(argv[3]) != "--extended")
      throw std::invalid_argument("unknown replay option");
    const bool extended = argc == 4;
    const auto original = cbf::load_configuration(argv[1]);
    cbf::ProblemSnapshotV2 captured;
    std::ifstream input(argv[2]);
    if (!cbf::read_problem_snapshot(input, captured))
      throw std::invalid_argument("invalid/truncated hard-constraint snapshot");
    if (captured.preparation_status != cbf::Status::solved_hard) {
      std::cout << "{\"cycle\":" << captured.cycle << ",\"preparation_status\":\""
                << cbf::name(captured.preparation_status)
                << "\",\"replayed\":false}\n";
      return 0;
    }
    for (auto backend : {cbf::Backend::osqp, cbf::Backend::qpoases, cbf::Backend::piqp}) {
      auto config = original;
      config.backend = backend;
      if (extended) {
        config.osqp_iterations = 20000;
        config.qpoases_recalculations = 1000;
        config.piqp_iterations = 200;
      }
      cbf::QpProblem problem(config);
      auto solver = cbf::make_solver(config, problem);
      const auto assembly = problem.assemble(captured.rows.data(), captured.count,
                                             captured.nominal, captured.bounds);
      cbf::SolverRun native;
      native.status = assembly;
      if (assembly == cbf::Status::solved_hard)
        native = solver->solve(problem);
      const auto result = cbf::verify_solution(problem, native.status, native.iterations,
                                               captured.nominal);
      std::cout.precision(17);
      std::cout << "{\"cycle\":" << captured.cycle << ",\"source_ns\":" << captured.source_ns
                << ",\"backend\":\"" << cbf::name(backend) << "\",\"rows\":"
                << captured.count << ",\"extended_iteration_budget\":"
                << (extended ? "true" : "false") << ",\"solver_status\":\""
                << cbf::name(native.status) << "\",\"verified_status\":\""
                << cbf::name(result.status) << "\",\"accepted\":"
                << (result.accepted() ? "true" : "false") << ",\"iterations\":"
                << result.iterations << ",\"max_hard_violation\":"
                << result.max_hard_violation << "}\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
