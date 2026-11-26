#pragma once
#include <iosfwd>

#include "cbf_safety_filter/types.hpp"

namespace cbf_safety_filter {
// Versioned hard-constraint problem snapshot for offline replay.
struct ProblemSnapshotV2 {
  std::array<Row, kMaxRows> rows{};
  int count{0};
  Vec7 nominal{Vec7::Zero()};
  TorqueBounds bounds{Vec7::Zero(), Vec7::Zero()};
  uint64_t source_ns{0}, cycle{0};
  Status preparation_status{Status::invalid_model};
};
bool read_problem_snapshot(std::istream&, ProblemSnapshotV2&);
}  // namespace cbf_safety_filter
