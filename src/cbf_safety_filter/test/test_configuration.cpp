#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>

#include "cbf_safety_filter/configuration.hpp"
namespace cbf = cbf_safety_filter;
TEST(Configuration, ShippedFileAndAllBackends) {
  const auto c = cbf::load_configuration(CBF_TEST_CONFIG);
  EXPECT_EQ(std::string(cbf::name(c.backend)),
            YAML::LoadFile(CBF_TEST_CONFIG)["solver"]["backend"].as<std::string>());
  EXPECT_TRUE(c.uses_body_volumes);
  EXPECT_TRUE(c.capsules.empty());
  EXPECT_EQ(c.body_spheres.size(), 15u);
  EXPECT_TRUE(c.position.enabled);
  for (auto backend : {cbf::Backend::osqp, cbf::Backend::qpoases, cbf::Backend::piqp}) {
    auto config = c;
    config.backend = backend;
    config.uses_body_volumes = false;
    config.recovery_torque_method = cbf::RecoveryTorqueMethod::projection;
    config.table.enabled = false;
    EXPECT_NO_THROW(config.validate());
  }
}
TEST(Configuration, RejectsInvalidCapacitiesPenaltiesAndUnknownKeys) {
  auto c = cbf::load_configuration(CBF_TEST_CONFIG);
  c.max_constraints = 1;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = cbf::load_configuration(CBF_TEST_CONFIG);
  const auto path = std::filesystem::temp_directory_path() / "cbf_invalid_config.yaml";
  std::ofstream(path) << "unknown: true\n";
  EXPECT_THROW(cbf::load_configuration(path.string()), std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(Configuration, InterArmReferenceBandOverridesWithoutChangingLocalBands) {
  auto c = cbf::load_configuration(CBF_TEST_CONFIG);
  EXPECT_DOUBLE_EQ(c.inter_arm_activation(), c.boundary_activation);
  c.inter_arm_boundary_activation = .1;
  EXPECT_DOUBLE_EQ(c.inter_arm_activation(), .1);
  EXPECT_DOUBLE_EQ(c.boundary_activation, .03);
  c.inter_arm_boundary_activation = -1;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(Configuration, SharedDualProfileSelectsDistinctGeometryAndHardJointLimits) {
  const auto path =
      std::filesystem::path(CBF_TEST_CONFIG).parent_path().parent_path() / "dual_panda/cbf.yaml";
  EXPECT_THROW(cbf::load_configuration(path.string()), std::invalid_argument);
  EXPECT_THROW(cbf::load_configuration(path.string(), "unknown"), std::invalid_argument);
  const auto left = cbf::load_configuration(path.string(), "left");
  const auto right = cbf::load_configuration(path.string(), "right");
  EXPECT_DOUBLE_EQ(left.world_from_base.translation().y(), 0.);
  EXPECT_DOUBLE_EQ(right.world_from_base.translation().y(), -.75);
  for (const auto* c : {&left, &right}) {
    EXPECT_TRUE(c->inter_arm.enabled);
    EXPECT_TRUE(c->uses_body_volumes);
    EXPECT_TRUE(c->capsules.empty());
    EXPECT_EQ(c->body_spheres.size(), 15u);
    EXPECT_DOUBLE_EQ(c->peer_max_age_us, 10000);
    EXPECT_TRUE(c->velocity.enabled);
    EXPECT_TRUE(c->acceleration.enabled);
    const cbf::Vec7 expected = (cbf::Vec7() << 15., 7.5, 10., 12.5, 15., 20., 20.).finished();
    EXPECT_TRUE(c->acceleration_limits.isApprox(expected));
  }
}

TEST(Configuration, SharedGeometryRejectsAmbiguousOrIncompleteSelection) {
  auto config = YAML::LoadFile(CBF_TEST_CONFIG);
  const auto geometry =
      std::filesystem::path(CBF_TEST_CONFIG).parent_path() / "geometry_panda_umi.yaml";
  config["geometry"]["file"] = geometry.string();
  config["geometry"]["files_by_controller"]["left"] = geometry.string();
  const auto path = std::filesystem::temp_directory_path() / "cbf_ambiguous_geometry.yaml";
  const auto write = [&]() { std::ofstream(path) << config; };
  write();
  EXPECT_THROW(cbf::load_configuration(path.string(), "left"), std::invalid_argument);
  config["geometry"].remove("file");
  write();
  EXPECT_THROW(cbf::load_configuration(path.string(), "left"), std::invalid_argument);
  config["geometry"]["files_by_controller"]["right"] = "";
  write();
  EXPECT_THROW(cbf::load_configuration(path.string(), "right"), std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(Configuration, DualProfileSelectsProjectionWithHardRecoveryAndExistingBudgets) {
  const auto root =
      std::filesystem::path(CBF_TEST_CONFIG).parent_path().parent_path() / "dual_panda";
  auto recovery = cbf::load_configuration((root / "cbf.yaml").string(), "left");
  EXPECT_EQ(recovery.recovery_torque_method, cbf::RecoveryTorqueMethod::projection);
  EXPECT_EQ(recovery.max_constraints, 512);
  EXPECT_DOUBLE_EQ(recovery.max_filter_time_us, 800);
  EXPECT_DOUBLE_EQ(recovery.max_callback_time_us, 800);
}

TEST(Configuration, SingleProfileUsesBodyVolumesWithoutPeerDependency) {
  const auto single = cbf::load_configuration(CBF_TEST_CONFIG);
  EXPECT_FALSE(single.inter_arm.enabled);
  EXPECT_DOUBLE_EQ(single.table.height, .127);
  EXPECT_DOUBLE_EQ(single.max_filter_time_us, 600);
  int table_rows = 0, moving_finger_volumes = 0;
  for (const auto& volume : single.body_spheres) {
    if (volume.used_for_capsule_table())
      table_rows += volume.span.isZero(0) ? 1 : 2;
    moving_finger_volumes += !volume.opening_axis.isZero(0);
  }
  EXPECT_EQ(table_rows, 24);
  EXPECT_EQ(moving_finger_volumes, 4);
}
TEST(Configuration, InheritanceRejectsCyclesAndPreservesNestedValidation) {
  const auto root = std::filesystem::temp_directory_path() / "cbf_profile_inheritance_test";
  std::filesystem::create_directories(root);
  std::ofstream(root / "a.yaml") << "extends: b.yaml\n";
  std::ofstream(root / "b.yaml") << "extends: a.yaml\n";
  EXPECT_THROW(cbf::load_configuration((root / "a.yaml").string()), std::invalid_argument);
  std::ofstream(root / "a.yaml") << "extends: " << CBF_TEST_CONFIG
                                 << "\nexecution:\n  misspelled: true\n";
  EXPECT_THROW(cbf::load_configuration((root / "a.yaml").string()), std::invalid_argument);
  std::ofstream(root / "a.yaml")
      << "extends: " << CBF_TEST_CONFIG
      << "\nexecution:\n  max_constraints: 256\n  max_constraints: 512\n";
  EXPECT_THROW(cbf::load_configuration((root / "a.yaml").string()), std::invalid_argument);
  std::filesystem::remove_all(root);
}

TEST(Configuration, SafetyTorqueComputationMethodIsExplicitAndValidated) {
  const std::filesystem::path base = CBF_TEST_CONFIG;
  const auto loaded = cbf::load_configuration(base.string());
  EXPECT_EQ(loaded.recovery_torque_method, cbf::RecoveryTorqueMethod::native_qp);
  const auto path = std::filesystem::temp_directory_path() / "cbf_torque_method_test.yaml";
  for (const auto* method : {"projection", "native_qp", "unknown"}) {
    std::ofstream(path) << "extends: " << base.string()
                        << "\nexecution:\n  safety_torque_computation_method: " << method << '\n';
    if (std::string(method) == "unknown")
      EXPECT_THROW(cbf::load_configuration(path.string()), std::invalid_argument);
    else
      EXPECT_NO_THROW(cbf::load_configuration(path.string()));
  }
  std::filesystem::remove(path);
  auto invalid = loaded;
  invalid.recovery_braking_gain = 0;
  EXPECT_THROW(invalid.validate(), std::invalid_argument);
}
