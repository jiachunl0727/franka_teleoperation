#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>
#include "cbf_safety_filter/qp_solver.hpp"
namespace cbf = cbf_safety_filter;
namespace {
using Clock = std::chrono::steady_clock;
double us(Clock::time_point t) { return std::chrono::duration<double, std::micro>(Clock::now()-t).count(); }
void statistics(std::vector<double> v) {
  std::sort(v.begin(),v.end());
  std::cout << ',' << std::accumulate(v.begin(),v.end(),0.)/v.size();
  for (double p : {.5,.95,.99,1.})
    std::cout << ',' << v[std::min(v.size()-1,static_cast<size_t>(p*(v.size()-1)))];
}
}
int main(int argc,char** argv) {
  const int repetitions=argc>1?std::stoi(argv[1]):30;
  if(repetitions<2||repetitions>10000) return 2;
  std::cout << "backend,cbf_rows,variables,repetitions,initialization_us,accepted,failures,max_iterations,max_hard_violation";
  for(const auto* phase:{"assembly","update","solve","verify","total"})
    for(const auto* stat:{"mean","p50","p95","p99","max"}) std::cout << ',' << phase << '_' << stat << "_us";
  std::cout << '\n';
  for(auto backend:{cbf::Backend::osqp,cbf::Backend::qpoases,cbf::Backend::piqp})
    for(int count:{10,30,100,200,400}) {
      cbf::Configuration c; c.backend=backend; c.max_constraints=count;
      c.osqp_iterations=5000; c.qpoases_recalculations=1000; c.piqp_iterations=100;
      c.osqp_abs=c.osqp_rel=c.piqp_abs=c.piqp_rel=1e-8;
      const auto init=Clock::now(); cbf::QpProblem problem(c); auto solver=cbf::make_solver(c,problem);
      const double init_us=us(init);
      std::vector<cbf::Row> rows(count);
      for(int r=0;r<count;++r) { for(int j=0;j<7;++j) rows[r].g[j]=std::sin((r+1)*(j+1)*.71); rows[r].g.normalize(); rows[r].bound=.05+.002*(r%7); }
      cbf::TorqueBounds box{cbf::Vec7::Constant(-2),cbf::Vec7::Constant(2)};
      std::array<std::vector<double>,5> times; for(auto& v:times) v.reserve(repetitions);
      int accepted=0,failures=0,max_iter=0; double max_violation=0;
      for(int k=0;k<repetitions;++k) {
        auto t=Clock::now(); auto status=problem.assemble(rows.data(),count,cbf::Vec7::Zero(),box); times[0].push_back(us(t));
        cbf::SolverRun raw; raw.status=status;
        if(status==cbf::Status::solved_hard) { t=Clock::now(); raw=solver->solve(problem); times[1].push_back(raw.update_us); times[2].push_back(raw.solve_us); }
        else { times[1].push_back(0); times[2].push_back(0); }
        t=Clock::now(); auto result=cbf::verify_solution(problem,raw.status,raw.iterations,cbf::Vec7::Zero()); times[3].push_back(us(t)); times[4].push_back(std::accumulate(times.begin(),times.end()-1,0.,[](double a,const auto& v){return a+v.back();}));
        accepted+=result.accepted(); failures+=!result.accepted(); max_iter=std::max(max_iter,result.iterations); max_violation=std::max(max_violation,result.max_hard_violation);
      }
      std::cout << cbf::name(backend) << ',' << count << ',' << problem.gradient.size() << ',' << repetitions << ',' << init_us << ',' << accepted << ',' << failures << ',' << max_iter << ',' << max_violation;
      for(auto& v:times) statistics(v); std::cout << '\n';
    }
}
