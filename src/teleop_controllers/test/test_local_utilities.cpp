#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <sstream>

#include "teleop_controllers/actuator_limits.hpp"
#include "teleop_controllers/inverse_dynamics_controller.hpp"
#include "teleop_controllers/legacy_pose.hpp"
#include "teleop_controllers/model_snapshot.hpp"
#include "teleop_controllers/urdf_model.hpp"
using namespace teleop_controllers;
namespace {
thread_local bool track_allocations = false;
thread_local size_t allocations = 0;
}  // namespace
extern "C" void* __real_malloc(size_t);
extern "C" void* __real_calloc(size_t, size_t);
extern "C" void* __real_realloc(void*, size_t);
extern "C" void* __wrap_malloc(size_t n) {
  if (track_allocations)
    ++allocations;
  return __real_malloc(n);
}
extern "C" void* __wrap_calloc(size_t n, size_t s) {
  if (track_allocations)
    ++allocations;
  return __real_calloc(n, s);
}
extern "C" void* __wrap_realloc(void* p, size_t n) {
  if (track_allocations)
    ++allocations;
  return __real_realloc(p, n);
}
void* operator new(size_t n) {
  if (track_allocations)
    ++allocations;
  if (auto p = std::malloc(n))
    return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }

TEST(ActuatorLimits, DefaultsUsePublishedFciCeilingsAndScalesAreValidated) {
  EXPECT_TRUE(panda_torque_limits().isApprox(
      (Vector7d() << 30.45, 30.45, 30.45, 30.45, 4.2, 4.2, 4.2).finished()));
  EXPECT_DOUBLE_EQ(panda_torque_rate_limit(), 350);
  EXPECT_TRUE(teleop_controllers::panda_torque_limits(.5).isApprox(
      (Vector7d() << 43.5, 43.5, 43.5, 43.5, 6, 6, 6).finished()));
  EXPECT_DOUBLE_EQ(teleop_controllers::panda_torque_rate_limit(.2), 200);
  teleop_controllers::ActuatorLimitScales scales;
  EXPECT_TRUE(scales.valid());
  for (double invalid : {0., -1., 1.01, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    scales = {};
    scales.torque = invalid;
    EXPECT_FALSE(scales.valid());
    scales = {};
    scales.torque_rate = invalid;
    EXPECT_FALSE(scales.valid());
  }
}

TEST(InverseDynamics, RealtimeComputationDoesNotAllocate) {
  ArmState s;
  s.q = Eigen::Map<const Vector7>(kHome.data());
  s.jacobian.leftCols<6>().setIdentity();
  s.jacobian.col(6).setConstant(.1);
  InverseDynamicsParameters p;
  InverseDynamicsController law(p);
  const CartesianReference target;
  law.compute(s, target, .001);
  bool valid = true;
  allocations = 0;
  track_allocations = true;
  for (int i = 0; i < 1000; ++i)
    valid &= law.compute(s, target, .001).fault == StateFault::none;
  track_allocations = false;
  EXPECT_TRUE(valid);
  EXPECT_EQ(allocations, size_t{0});
}

TEST(UrdfModel, RealtimeEvaluationDoesNotAllocate) {
  UrdfPandaModel model(TELEOP_URDF_MODEL_FILE);
  UrdfModelOutput output;
  const Vector7 q = Eigen::Map<const Vector7>(kHome.data());
  const Vector7 dq = Vector7::Constant(.2);
  const auto tool = model.standard_flange_to_ee();
  bool valid = true;
  allocations = 0;
  track_allocations = true;
  for (int i = 0; i < 1000; ++i)
    valid &= model.evaluate(q, dq, tool, output);
  track_allocations = false;
  EXPECT_TRUE(valid);
  EXPECT_EQ(allocations, size_t{0});
}

TEST(ModelConversion, LegacyRoundedEndEffectorRotationPassesOnlyAfterBoundedConversion) {
  Eigen::Affine3d rounded = Eigen::Affine3d::Identity();
  const double c = static_cast<float>(0.7071);
  rounded.linear() << c, c, 0, -c, c, 0, 0, 0, 1;
  rounded.translation() << .1, .2, .1034;
  // Exercise an arbitrary base/flange rotation as in the hardware report.
  rounded = Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()) * rounded;
  const Eigen::Vector3d position = rounded.translation();
  ASSERT_TRUE(teleop_controllers::orthonormalize_legacy_pose(rounded));
  EXPECT_TRUE(rounded.translation().isApprox(position, 1e-14));
  EXPECT_NEAR(rounded.linear().determinant(), 1., 1e-12);
}

TEST(ModelConversion, LegacyPoseConversionRejectsMalformedAndDistortedTransforms) {
  for (int scenario = 0; scenario < 7; ++scenario) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    if (scenario == 0)
      pose.linear()(0, 0) = 1.001;
    if (scenario == 1)
      pose.linear()(0, 0) = -1.;
    if (scenario == 2)
      pose.linear()(0, 1) = .001;
    if (scenario == 3)
      pose.matrix()(3, 3) = 2.;
    if (scenario == 4)
      pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
    if (scenario == 5)
      pose.linear()(0, 0) = 0.;
    if (scenario == 6)
      pose.linear()(0, 0) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(teleop_controllers::orthonormalize_legacy_pose(pose)) << scenario;
  }
}

TEST(ModelConversion, LegacyPoseConversionDoesNotAllocate) {
  Eigen::Affine3d input = Eigen::Affine3d::Identity();
  input.linear() << .7071, .7071, 0, -.7071, .7071, 0, 0, 0, 1;
  bool valid = true;
  allocations = 0;
  track_allocations = true;
  for (int n = 0; n < 20; ++n) {
    Eigen::Affine3d pose = input;
    valid &= teleop_controllers::orthonormalize_legacy_pose(pose);
  }
  track_allocations = false;
  EXPECT_TRUE(valid);
  EXPECT_EQ(allocations, size_t{0});
}

TEST(ActuatorLimits, ExactIntervalIntersectionHasNoControlPolicy) {
  const auto caps = teleop_controllers::panda_torque_limits();
  for (double sign : {-1., 1.}) {
    const Vector7d previous = sign * (caps.array() - .1).matrix();
    const auto interval = teleop_controllers::actuator_bounds(previous, caps, 350, .001);
    ASSERT_TRUE(interval.has_value());
    const Vector7d torque = interval->clamp(Vector7d::Constant(sign * 100));
    EXPECT_TRUE(torque.isApprox(sign * caps));
    EXPECT_LE((torque - previous).cwiseAbs().maxCoeff(), .35 + 1e-12);
  }
  EXPECT_FALSE(teleop_controllers::actuator_bounds(caps + Vector7d::Ones(), caps, 350, .001));
  EXPECT_FALSE(teleop_controllers::actuator_bounds(Vector7d::Zero(), caps, 350, 0));
}

TEST(ModelSnapshot, RecordedHardwareDataSurvivesArchitectureRemoval) {
  size_t count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(TELEOP_RECORDED_MODEL_DIR)) {
    if (entry.path().extension() != ".txt")
      continue;
    SCOPED_TRACE(entry.path().string());
    std::ifstream input(entry.path());
    teleop_controllers::ModelSnapshot saved;
    ASSERT_TRUE(teleop_controllers::read_model_snapshot(input, saved));
    ASSERT_TRUE(saved.state.mass.allFinite());
    ASSERT_TRUE(saved.state.previous_torque.allFinite());
    std::stringstream round_trip;
    ASSERT_TRUE(teleop_controllers::write_model_snapshot(round_trip, saved));
    teleop_controllers::ModelSnapshot decoded;
    ASSERT_TRUE(teleop_controllers::read_model_snapshot(round_trip, decoded));
    EXPECT_EQ(saved.cycle, decoded.cycle);
    EXPECT_TRUE(saved.state.mass.isApprox(decoded.state.mass, 0));
    EXPECT_TRUE(saved.state.q.isApprox(decoded.state.q, 0));
    EXPECT_TRUE(saved.nominal.isApprox(decoded.nominal, 0));
    for (int frame = 0; frame < kModelFrames; ++frame) {
      EXPECT_TRUE(saved.state.frame_pose[frame].matrix().isApprox(
          decoded.state.frame_pose[frame].matrix(), 0));
      EXPECT_TRUE(
          saved.state.frame_jacobian[frame].isApprox(decoded.state.frame_jacobian[frame], 0));
    }
    ++count;
  }
  EXPECT_GE(count, 20u);
  std::stringstream truncated("CBF_MODEL_V1 0 1 .001\n0");
  teleop_controllers::ModelSnapshot untouched;
  untouched.cycle = 99;
  EXPECT_FALSE(teleop_controllers::read_model_snapshot(truncated, untouched));
  EXPECT_EQ(untouched.cycle, 99u);
}
