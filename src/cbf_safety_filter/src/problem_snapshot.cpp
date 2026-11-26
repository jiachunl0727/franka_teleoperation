#include "cbf_safety_filter/problem_snapshot.hpp"

#include <cmath>
#include <istream>
#include <string>

namespace cbf_safety_filter {
bool read_problem_snapshot(std::istream& stream, ProblemSnapshotV2& p) {
  ProblemSnapshotV2 parsed;
  std::string version;
  int status;
  if (!(stream >> version >> parsed.count >> parsed.source_ns >> parsed.cycle >> status) ||
      version != "CBF_PROBLEM_V2" || parsed.count < 0 || parsed.count > kMaxRows || status < 0 ||
      status > static_cast<int>(Status::verified_recovery))
    return false;
  parsed.preparation_status = static_cast<Status>(status);
  for (auto* v : {&parsed.nominal, &parsed.bounds.lower, &parsed.bounds.upper}) {
    for (int j = 0; j < 7; ++j)
      stream >> (*v)[j];
    if (!v->allFinite())
      return false;
  }
  if ((parsed.bounds.lower.array() > parsed.bounds.upper.array()).any())
    return false;
  for (int i = 0; i < parsed.count; ++i) {
    auto& r = parsed.rows[i];
    int family;
    if (!(stream >> family >> r.h >> r.psi >> r.bound) || family < 0 ||
        family > static_cast<int>(Family::peer_motion))
      return false;
    r.family = static_cast<Family>(family);
    for (int j = 0; j < 7; ++j)
      stream >> r.g[j];
    if (!r.g.allFinite() || !std::isfinite(r.bound) || !std::isfinite(r.h) ||
        !std::isfinite(r.psi))
      return false;
  }
  if (!stream)
    return false;
  p = parsed;
  return true;
}
}  // namespace cbf_safety_filter
