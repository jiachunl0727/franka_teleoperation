#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "cbf_safety_filter/safety_filter.hpp"
namespace cbf = cbf_safety_filter;
namespace {
struct Recorded {
  cbf::Vec7 q, dq, coriolis, previous, nominal;
  cbf::Mat7 mass;
  std::array<Eigen::Affine3d, 9> poses;
  std::array<cbf::Jacobian, 9> jacobians;
  bool read(const std::filesystem::path& file) {
    std::ifstream stream(file);
    std::string version;
    int arm;
    uint64_t cycle;
    double dt;
    stream >> version >> arm >> cycle >> dt;
    if (version != "CBF_MODEL_V1" && version != "ROBOT_MODEL_V1")
      return false;
    const auto values = [&](auto& v) {
      for (int k = 0; k < v.size(); ++k)
        stream >> v.data()[k];
    };
    values(q);
    values(dq);
    values(coriolis);
    values(previous);
    values(nominal);
    values(mass);
    for (int f = 0; f < 9; ++f) {
      values(poses[f].matrix());
      values(jacobians[f]);
    }
    return bool(stream);
  }
  cbf::ModelView view() const { return {q, dq, mass, coriolis, poses, jacobians, 1}; }
};
}  // namespace
TEST(RecordedIncident, EachBackendRejectsOrIndependentlyVerifiesEveryHistoricalCandidate) {
  size_t samples = 0, postclip_rejections = 0;
  double largest_locked_wrist_disagreement = 0;
  for (const auto& file : std::filesystem::recursive_directory_iterator(CBF_RECORDED_DIR)) {
    if (file.path().extension() != ".txt")
      continue;
    Recorded m;
    if (!m.read(file.path()))
      continue;
    SCOPED_TRACE(file.path().string());
    ++samples;
    const cbf::Vec7 model_acceleration = m.mass.llt().solve(m.previous - m.coriolis);
    // Adversarial plant: joints 5-7 are mechanically locked; proximal motion
    // obeys the constrained coupled mass block. The full model is not this plant.
    cbf::Vec7 actual_acceleration = cbf::Vec7::Zero();
    actual_acceleration.head<4>() =
        m.mass.topLeftCorner<4, 4>().llt().solve((m.previous - m.coriolis).head<4>());
    largest_locked_wrist_disagreement =
        std::max(largest_locked_wrist_disagreement,
                 (model_acceleration - actual_acceleration).tail<3>().norm());
    for (auto backend : {cbf::Backend::osqp, cbf::Backend::qpoases, cbf::Backend::piqp}) {
        cbf::Configuration c;
        c.backend = backend;
        c.position.enabled = c.velocity.enabled = true;
        c.max_constraints = 28;
        c.max_filter_time_us = 1e8;
        c.osqp_iterations = 10000;
        c.piqp_iterations = 200;
        cbf::SafetyFilter filter(c);
        const cbf::Vec7 caps =
            (cbf::Vec7() << 30.45, 30.45, 30.45, 30.45, 4.2, 4.2, 4.2).finished();
        cbf::TorqueBounds bounds{(-caps).cwiseMax((m.previous.array() - .35).matrix()),
                                 caps.cwiseMin((m.previous.array() + .35).matrix())};
        const auto result = filter.compute(m.view(), m.nominal, bounds, .001, 1);
        if (result.accepted()) {
          EXPECT_LE(result.max_hard_violation, c.hard_tolerance);
          const cbf::Vec7 outgoing = result.torque.cwiseMax(bounds.lower).cwiseMin(bounds.upper);
          if (!filter.verify_command(outgoing)) {
            ++postclip_rejections;
            // Clipping a numerical bound residual can amplify through small
            // wrist inertia. The owner must reject it, not trust solver success.
            cbf::ConstraintBuilder independent_rows(c);
            const auto& rows = independent_rows.build(m.view(), .001, 1);
            double violation = 0;
            for (int row = 0; row < rows.count; ++row)
              violation =
                  std::max(violation, rows.rows[row].g.dot(outgoing) - rows.rows[row].bound);
            EXPECT_GT(violation, c.hard_tolerance);
          }
        } else
          EXPECT_FALSE(filter.verify_command(m.previous));
      }
  }
  EXPECT_GE(samples, 20u);
  EXPECT_GT(largest_locked_wrist_disagreement, 1.);
  RecordProperty("largest_locked_wrist_model_acceleration_disagreement",
                 largest_locked_wrist_disagreement);
  RecordProperty("postclip_rejections", static_cast<int>(postclip_rejections));
}
