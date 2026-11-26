// Frozen dual-arm independent-policy replay benchmark. No ROS, Robot, sockets or command output.
#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "cbf_safety_filter/safety_filter.hpp"
#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/urdf_model.hpp"

namespace cbf = cbf_safety_filter;
namespace ft = teleop_controllers;
using Sample = std::array<double, 54>;
using Clock = std::chrono::steady_clock;
uint64_t ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
}
std::vector<Sample> load(const char* path) {
  std::ifstream in(path, std::ios::binary);
  char magic[8];
  uint64_t count = 0, fields = 0;
  in.read(magic, 8);
  in.read(reinterpret_cast<char*>(&count), 8);
  in.read(reinterpret_cast<char*>(&fields), 8);
  if (!in || std::memcmp(magic, "AVOID01\n", 8) || fields != 54 || count > 10000000)
    throw std::runtime_error("invalid recorded stream");
  std::vector<Sample> rows(count);
  in.read(reinterpret_cast<char*>(rows.data()), count * sizeof(Sample));
  if (!in || !count)
    throw std::runtime_error("empty/truncated stream");
  for (size_t i = 0; i < rows.size(); ++i)
    if (!Eigen::Map<const Eigen::Matrix<double, 54, 1>>(rows[i].data()).allFinite() ||
        (i && (rows[i][0] <= rows[i - 1][0] || rows[i][52] <= rows[i - 1][52])))
      throw std::runtime_error("nonfinite/unordered samples");
  return rows;
}
int main(int argc, char** argv) try {
  if (argc != 6 && argc != 7)
    throw std::runtime_error(
        "usage: dual_avoidance_replay LEFT_YAML RIGHT_YAML URDF LEFT_BIN RIGHT_BIN [direct]");
  const bool direct = argc == 7 && std::string(argv[6]) == "direct";
  if (argc == 7 && !direct)
    throw std::runtime_error("unknown replay mode");
  const ft::UrdfPandaModel model(argv[3]);
  std::array<cbf::Configuration, 2> configs{cbf::load_configuration(argv[1], "left"),
                                            cbf::load_configuration(argv[2], "right")};
  std::array<std::vector<Sample>, 2> data{load(argv[4]), load(argv[5])};
  std::cout << std::setprecision(17) << std::boolalpha;
  for (int arm = 0; arm < 2; ++arm) {
    auto c = configs[arm];
    // Offline measurement only: report time rather than letting host load hide
    // numerical equivalence. All original geometry/actuator/freshness gates stay.
    c.max_filter_time_us = c.max_model_age_us = 1000000;
    auto filter = std::make_unique<cbf::SafetyFilter>(c);
    size_t other = 0;
    for (const auto& row : data[arm]) {
      const auto& peers = data[1 - arm];
      if (row[52] < peers.front()[52] || row[52] > peers.back()[52])
        continue;
      while (other + 1 < peers.size() && peers[other + 1][52] <= row[52])
        ++other;
      const auto& pr = peers[other];
      ft::UrdfModelOutput own, remote;
      if (!model.evaluate(Eigen::Map<const ft::Vector7>(row.data() + 2),
                          Eigen::Map<const ft::Vector7>(row.data() + 9),
                          model.standard_flange_to_ee(), own) ||
          !model.evaluate(Eigen::Map<const ft::Vector7>(pr.data() + 2),
                          Eigen::Map<const ft::Vector7>(pr.data() + 9),
                          model.standard_flange_to_ee(), remote))
        throw std::runtime_error("invalid model");
      const auto bounds =
          ft::actuator_bounds(Eigen::Map<const ft::Vector7>(row.data() + 16),
                              Eigen::Map<const ft::Vector7>(row.data() + 37), row[44], row[1]);
      if (!bounds)
        throw std::runtime_error("invalid interval");
      const auto& m = own.state;
      const auto& p = remote.state;
      const auto stamp = ns();
      cbf::ModelView view{m.q, m.dq, m.mass, m.coriolis, m.frame_pose, m.frame_jacobian, stamp};
      const auto age_ns = static_cast<uint64_t>(row[52] - pr[52]);
      const cbf::ModelView pv{
          p.q, p.dq, p.mass, p.coriolis, p.frame_pose, p.frame_jacobian, stamp - age_ns};
      auto peer = cbf::collision_snapshot(pv, configs[1 - arm]);
      const auto begin = ns();
      view.timestamp_ns = begin;
      peer.timestamp_ns = begin - age_ns;
#ifdef CBF_PEER_SNAPSHOT_EXCHANGE
      if (!direct) {
        const cbf::PeerExchange exchange{const_cast<cbf::PeerSnapshot*>(&peer),
                                         [](void* context, const cbf::PeerSnapshot&) noexcept {
                                           return static_cast<const cbf::PeerSnapshot*>(context);
                                         }};
        filter->prepare(view, {bounds->lower, bounds->upper}, row[1], ns(), nullptr, &exchange);
      } else
#endif
      {
        const auto snapshot = cbf::collision_snapshot(view, c);
        if (!snapshot.valid)
          throw std::runtime_error("invalid own snapshot");
        filter->prepare(view, {bounds->lower, bounds->upper}, row[1], direct ? begin : ns(), &peer);
      }
      const auto& r = filter->solve_prepared(Eigen::Map<const ft::Vector7>(row.data() + 23));
      const bool verified = r.accepted() && filter->verify_command(r.torque);
      const auto elapsed = (ns() - begin) * 1e-3;
      std::cout << "{\"arm\":" << arm << ",\"cycle\":" << uint64_t(row[0])
                << ",\"peer_age_us\":" << age_ns * 1e-3 << ",\"status\":\"" << cbf::name(r.status)
                << "\",\"verified\":" << verified << ",\"rows\":" << r.rows
                << ",\"kinematics_us\":" << r.kinematics_us
                << ",\"constraints_us\":" << r.constraints_us << ",\"filter_us\":" << r.total_us
                << ",\"snapshot_filter_verify_us\":" << elapsed << ",\"torque\":[";
      for (int j = 0; j < 7; ++j)
        std::cout << (j ? "," : "") << r.torque[j];
      std::cout << ']';
#ifdef CBF_PEER_SNAPSHOT_EXCHANGE
      std::cout << ",\"group_pruned_pairs\":" << r.group_pruned_pairs
                << ",\"evaluated_peer_pairs\":" << r.evaluated_peer_pairs;
#endif
      std::cout << "}\n";
    }
  }
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
