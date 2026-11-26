#include <gtest/gtest.h>

#include <sstream>

#include "cbf_safety_filter/problem_snapshot.hpp"

namespace cbf = cbf_safety_filter;
TEST(ProblemSnapshot, RejectsTruncatedUnknownAndOutOfRangeFiles) {
  cbf::ProblemSnapshotV2 captured;
  for (const std::string text : {"BAD 0", "CBF_PROBLEM_V2 513", "CBF_PROBLEM_V2 -1",
                                 "CBF_PROBLEM_V2 1 1 1 1 1 1 1 1 0\n0 0"}) {
    std::istringstream stream(text);
    EXPECT_FALSE(cbf::read_problem_snapshot(stream, captured));
  }
}
