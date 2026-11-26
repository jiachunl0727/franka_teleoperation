#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>

#include "teleop_controllers/panda_model_fixture.hpp"
#include "teleop_controllers/urdf_model.hpp"

namespace ft = teleop_controllers;
namespace {
template <class T>
void read(std::istream& input, T& value) {
  for (int i = 0; i < value.size(); ++i)
    input >> value.data()[i];
}
}  // namespace

TEST(UrdfModel, MatchesUnmodifiedOscbfAcrossRecordedAndRandomStates) {
  ft::UrdfPandaModel model(TELEOP_URDF_MODEL_FILE);
  std::ifstream input(TELEOP_URDF_REFERENCE_FILE);
  std::string version;
  int count = 0;
  input >> version >> count;
  ASSERT_EQ(version, "OSCBF_REFERENCE_V1");
  ASSERT_EQ(count, 64);
  for (int i = 0; i < count; ++i) {
    SCOPED_TRACE(i);
    ft::Vector7d q, dq, coriolis, gravity;
    ft::Matrix7d mass;
    read(input, q);
    read(input, dq);
    read(input, mass);
    read(input, coriolis);
    read(input, gravity);
    ASSERT_TRUE(input);
    ft::UrdfModelOutput out;
    ASSERT_TRUE(model.evaluate(q, dq, model.standard_flange_to_ee(), out));
    EXPECT_LT((out.state.mass - mass).cwiseAbs().maxCoeff(), 1e-10);
    EXPECT_LT((out.state.coriolis - coriolis).cwiseAbs().maxCoeff(), 1e-10);
    EXPECT_LT((out.gravity - gravity).cwiseAbs().maxCoeff(), 1e-10);
    EXPECT_TRUE(out.state.mass.isApprox(out.state.mass.transpose(), 1e-12));
    EXPECT_EQ(Eigen::LLT<ft::Matrix7d>(out.state.mass).info(), Eigen::Success);
    const auto kinematics = ft::panda_model_fixture(q);
    for (int frame = 0; frame < 9; ++frame) {
      EXPECT_LT((out.state.frame_pose[frame].matrix() - kinematics.frame_pose[frame].matrix())
                    .cwiseAbs()
                    .maxCoeff(),
                2e-9);
      EXPECT_LT((out.state.frame_jacobian[frame] - kinematics.frame_jacobian[frame])
                    .cwiseAbs()
                    .maxCoeff(),
                2e-9);
    }
  }
}

TEST(UrdfModel, KeepsFixedBodyInertiasAndUsesConfiguredToolOnlyForKinematics) {
  ft::UrdfPandaModel model(TELEOP_URDF_MODEL_FILE);
  const ft::Vector7d q = ft::Vector7d::Constant(.3), v = ft::Vector7d::Constant(.2);
  ft::UrdfModelOutput standard, shifted;
  const auto tool = model.standard_flange_to_ee();
  ASSERT_TRUE(model.evaluate(q, v, tool, standard));
  EXPECT_NEAR(standard.state.mass(6, 6), .60002, 1e-12);
  auto moved = tool;
  moved.translation().x() += .05;
  ASSERT_TRUE(model.evaluate(q, v, moved, shifted));
  EXPECT_TRUE(standard.state.mass.isApprox(shifted.state.mass, 1e-14));
  EXPECT_TRUE(standard.state.coriolis.isApprox(shifted.state.coriolis, 1e-14));
  EXPECT_TRUE(standard.gravity.isApprox(shifted.gravity, 1e-14));
  EXPECT_GT((standard.state.frame_jacobian[8] - shifted.state.frame_jacobian[8]).norm(), .01);
}

TEST(UrdfModel, ZeroVelocityAndQuadraticVelocityScalingHaveCorrectBias) {
  ft::UrdfPandaModel model(TELEOP_URDF_MODEL_FILE);
  const ft::Vector7d q = ft::Vector7d::Constant(.3), v = ft::Vector7d::Constant(.2);
  ft::UrdfModelOutput a, b, zero;
  ASSERT_TRUE(model.evaluate(q, v, model.standard_flange_to_ee(), a));
  ASSERT_TRUE(model.evaluate(q, 2 * v, model.standard_flange_to_ee(), b));
  ASSERT_TRUE(model.evaluate(q, ft::Vector7d::Zero(), model.standard_flange_to_ee(), zero));
  EXPECT_TRUE(zero.state.coriolis.isZero(1e-14));
  EXPECT_TRUE(b.state.coriolis.isApprox(4 * a.state.coriolis, 1e-12));
  EXPECT_TRUE(a.gravity.isApprox(b.gravity, 1e-14));
}

TEST(UrdfModel, UmiAttachmentsAddPositiveInertiaWithoutChangingArmKinematics) {
  ft::UrdfPandaModel stock(TELEOP_URDF_MODEL_FILE), attached(TELEOP_UMI_MODEL_FILE);
  for (int sample = 0; sample < 8; ++sample) {
    const ft::Vector7d q = ft::Vector7d::Constant(.1 * sample);
    const ft::Vector7d v = ft::Vector7d::Constant(.03 * sample);
    ft::UrdfModelOutput a, b;
    ASSERT_TRUE(stock.evaluate(q, v, stock.standard_flange_to_ee(), a));
    ASSERT_TRUE(attached.evaluate(q, v, attached.standard_flange_to_ee(), b));
    for (int frame = 0; frame < 9; ++frame) {
      EXPECT_TRUE(
          a.state.frame_pose[frame].matrix().isApprox(b.state.frame_pose[frame].matrix(), 1e-13));
      EXPECT_TRUE(a.state.frame_jacobian[frame].isApprox(b.state.frame_jacobian[frame], 1e-13));
    }
    const ft::Matrix7d delta = b.state.mass - a.state.mass;
    EXPECT_GT(delta.norm(), 1e-5);
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<ft::Matrix7d>(delta).eigenvalues().minCoeff(), -1e-12);
    EXPECT_GT((b.gravity - a.gravity).norm(), .001);
    EXPECT_GT(b.state.mass(6, 6), .60002);
  }
}

TEST(UrdfModel, InvalidStateAndToolDoNotReturnAUsableModel) {
  ft::UrdfPandaModel model(TELEOP_URDF_MODEL_FILE);
  ft::Vector7d q = ft::Vector7d::Zero();
  ft::UrdfModelOutput output;
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(model.evaluate(q, ft::Vector7d::Zero(), model.standard_flange_to_ee(), output));
  q.setZero();
  auto tool = model.standard_flange_to_ee();
  tool.linear()(0, 0) = 2;
  EXPECT_FALSE(model.evaluate(q, q, tool, output));
}

TEST(UrdfModel, InvalidFilesAreRejectedAtConstruction) {
  EXPECT_THROW(ft::UrdfPandaModel("/missing/robot.urdf"), std::invalid_argument);
  const auto directory =
      std::filesystem::temp_directory_path() / ("teleop-urdf-test-" + std::to_string(getpid()));
  ASSERT_TRUE(std::filesystem::create_directory(directory));
  std::ifstream input(TELEOP_URDF_MODEL_FILE);
  const std::string original((std::istreambuf_iterator<char>(input)), {});
  for (int kind = 0; kind < 4; ++kind) {
    std::string bad = original;
    if (kind == 0)
      bad = "<robot>";
    if (kind == 1) {
      const auto p = bad.find("type=\"revolute\"");
      ASSERT_NE(p, std::string::npos);
      bad.replace(p, 15, "type=\"prismatic\"");
    }
    if (kind == 2) {
      const auto p = bad.find("ixx=\"0.1\"");
      ASSERT_NE(p, std::string::npos);
      bad.replace(p, 9, "ixx=\"-1\"");
    }
    if (kind == 3) {
      const auto p = bad.find("panda_joint7");
      ASSERT_NE(p, std::string::npos);
      bad.replace(p, 12, "other_joint7");
    }
    const auto path = directory / (std::to_string(kind) + ".urdf");
    {
      std::ofstream out(path);
      out << bad;
    }
    EXPECT_THROW(ft::UrdfPandaModel model(path.string()), std::invalid_argument);
  }
  std::filesystem::remove_all(directory);
}
