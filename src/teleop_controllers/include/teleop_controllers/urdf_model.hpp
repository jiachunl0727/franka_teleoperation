#pragma once

#include "teleop_controllers/robot_model.hpp"

namespace teleop_controllers {
struct UrdfModelOutput {
  RobotModelState state;
  Vector7d gravity{Vector7d::Zero()};
};

// Immutable after construction. XML/file work happens only in the constructor.
// Evaluation uses fixed-size storage and the same rigid-body energy model as
// pinned OSCBF, retaining inertia on fixed bodies even when their mass is zero.
class UrdfPandaModel {
 public:
  explicit UrdfPandaModel(const std::string& filename);
  bool evaluate(const Vector7d& q, const Vector7d& dq, const Eigen::Affine3d& flange_to_ee,
                UrdfModelOutput& output) const noexcept;
  const std::string& filename() const noexcept { return filename_; }
  const std::string& fingerprint() const noexcept { return fingerprint_; }
  static Eigen::Affine3d standard_flange_to_ee() noexcept;

 private:
  static constexpr int kMaxBodies = 32;
  struct Body {
    Eigen::Affine3d origin{Eigen::Affine3d::Identity()};
    Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()}, com{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};  // In link coordinates at CoM.
    double mass{0};
    int parent{-1}, joint{-1};
    unsigned ancestors{0};
  };
  std::array<Body, kMaxBodies> bodies_;
  std::array<int, 7> joint_bodies_{};
  int count_{0}, flange_{-1};
  std::string filename_, fingerprint_;
};
}  // namespace teleop_controllers
