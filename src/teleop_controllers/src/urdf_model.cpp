#include "teleop_controllers/urdf_model.hpp"

#include <tinyxml2.h>

#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace teleop_controllers {
namespace {
using Element = tinyxml2::XMLElement;
void require(bool condition, const char* message) {
  if (!condition)
    throw std::invalid_argument(std::string("invalid Panda URDF: ") + message);
}
std::string attribute(const Element* element, const char* key) {
  require(element && element->Attribute(key), "missing element/attribute");
  return element->Attribute(key);
}
double number(const Element* element, const char* key) {
  double result = 0;
  require(element && element->QueryDoubleAttribute(key, &result) == tinyxml2::XML_SUCCESS &&
              std::isfinite(result),
          "invalid numeric attribute");
  return result;
}
Eigen::Vector3d vector(const char* text, Eigen::Vector3d fallback = Eigen::Vector3d::Zero()) {
  if (!text)
    return fallback;
  std::istringstream input(text);
  Eigen::Vector3d result;
  require(bool(input >> result[0] >> result[1] >> result[2]) && result.allFinite(),
          "invalid vector");
  std::string extra;
  require(!(input >> extra), "extra vector components");
  return result;
}
Eigen::Affine3d origin(const Element* element) {
  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  if (!element)
    return result;
  const auto rpy = vector(element->Attribute("rpy"));
  result.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();
  result.translation() = vector(element->Attribute("xyz"));
  return result;
}
}  // namespace

UrdfPandaModel::UrdfPandaModel(const std::string& filename) : filename_(filename) {
  std::ifstream file(filename, std::ios::binary);
  require(bool(file), "cannot open model file");
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  uint64_t hash = 14695981039346656037ULL;
  for (unsigned char c : text)
    hash = (hash ^ c) * 1099511628211ULL;
  std::ostringstream fingerprint;
  fingerprint << std::hex << std::setw(16) << std::setfill('0') << hash;
  fingerprint_ = fingerprint.str();
  tinyxml2::XMLDocument xml;
  require(xml.Parse(text.c_str(), text.size()) == tinyxml2::XML_SUCCESS, "malformed XML");
  const auto* robot = xml.FirstChildElement("robot");
  require(robot != nullptr, "missing robot");
  std::map<std::string, const Element*> links;
  for (auto* e = robot->FirstChildElement("link"); e; e = e->NextSiblingElement("link"))
    require(links.emplace(attribute(e, "name"), e).second, "duplicate link");
  require(!links.empty() && links.size() <= kMaxBodies, "unsupported link count");
  struct Joint {
    std::string name, parent, child, type;
    const Element* xml;
  };
  std::vector<Joint> joints;
  std::set<std::string> names, children;
  for (auto* e = robot->FirstChildElement("joint"); e; e = e->NextSiblingElement("joint")) {
    Joint j{attribute(e, "name"), attribute(e->FirstChildElement("parent"), "link"),
            attribute(e->FirstChildElement("child"), "link"), attribute(e, "type"), e};
    require(names.insert(j.name).second && children.insert(j.child).second,
            "duplicate joint/parent");
    require(links.count(j.parent) && links.count(j.child), "unknown joint link");
    require(j.type == "fixed" || j.type == "revolute", "only fixed/revolute joints supported");
    joints.push_back(j);
  }
  require(links.count("panda_link0") && !children.count("panda_link0"), "wrong root link");
  joint_bodies_.fill(-1);
  std::set<std::string> visited;
  std::function<void(const std::string&, int, const Joint*)> append;
  append = [&](const std::string& name, int parent, const Joint* joint) {
    require(visited.insert(name).second, "cycle in model");
    require(count_ < kMaxBodies, "too many bodies");
    const int index = count_++;
    auto& b = bodies_[index];
    b.parent = parent;
    if (joint) {
      b.origin = origin(joint->xml->FirstChildElement("origin"));
      b.ancestors = bodies_[parent].ancestors;
      if (joint->type == "revolute") {
        for (int j = 0; j < 7; ++j)
          if (joint->name == "panda_joint" + std::to_string(j + 1))
            b.joint = j;
        require(b.joint >= 0 && joint_bodies_[b.joint] < 0, "wrong active joint name");
        require(name == "panda_link" + std::to_string(b.joint + 1) &&
                    joint->parent == "panda_link" + std::to_string(b.joint),
                "wrong active chain");
        const auto* axis = joint->xml->FirstChildElement("axis");
        b.axis = vector(axis ? axis->Attribute("xyz") : nullptr, Eigen::Vector3d::UnitX());
        require(b.axis.norm() > 1e-12, "zero active joint axis");
        b.axis.normalize();
        b.ancestors |= 1u << b.joint;
        joint_bodies_[b.joint] = index;
      }
    }
    const auto* inertial = links.at(name)->FirstChildElement("inertial");
    if (inertial) {
      b.mass = number(inertial->FirstChildElement("mass"), "value");
      require(b.mass >= 0, "negative mass");
      const auto transform = origin(inertial->FirstChildElement("origin"));
      b.com = transform.translation();
      const auto* i = inertial->FirstChildElement("inertia");
      Eigen::Matrix3d tensor;
      tensor << number(i, "ixx"), number(i, "ixy"), number(i, "ixz"), number(i, "ixy"),
          number(i, "iyy"), number(i, "iyz"), number(i, "ixz"), number(i, "iyz"), number(i, "izz");
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(tensor);
      require(eigen.info() == Eigen::Success && eigen.eigenvalues().minCoeff() >= -1e-12,
              "indefinite link inertia");
      b.inertia = transform.linear() * tensor * transform.linear().transpose();
    }
    if (name == "panda_link8")
      flange_ = index;
    for (const auto& child : joints)
      if (child.parent == name)
        append(child.child, index, &child);
  };
  append("panda_link0", -1, nullptr);
  require(visited.size() == links.size(), "disconnected model");
  for (int j : joint_bodies_)
    require(j >= 0, "missing active joint");
  require(flange_ >= 0 && bodies_[flange_].ancestors == 127 && bodies_[flange_].joint < 0,
          "missing/invalid flange");
  UrdfModelOutput check;
  require(evaluate(Vector7d::Zero(), Vector7d::Zero(), standard_flange_to_ee(), check) &&
              Eigen::LLT<Matrix7d>(check.state.mass).info() == Eigen::Success,
          "singular model inertia");
}

Eigen::Affine3d UrdfPandaModel::standard_flange_to_ee() noexcept {
  return Eigen::Affine3d(Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()) *
                         Eigen::Translation3d(0, 0, .1034));
}

bool UrdfPandaModel::evaluate(const Vector7d& q, const Vector7d& dq,
                              const Eigen::Affine3d& flange_to_ee,
                              UrdfModelOutput& out) const noexcept {
  if (!q.allFinite() || !dq.allFinite() || !flange_to_ee.matrix().allFinite() ||
      !flange_to_ee.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-10) ||
      !(flange_to_ee.linear().transpose() * flange_to_ee.linear()).isIdentity(1e-8) ||
      std::abs(flange_to_ee.linear().determinant() - 1.) > 1e-8)
    return false;
  struct State {
    Eigen::Affine3d pose;
    Eigen::Vector3d omega, alpha, acceleration;
  };
  std::array<State, kMaxBodies> states;
  std::array<Eigen::Vector3d, 7> axes, origins;
  states[0].pose.setIdentity();
  states[0].omega.setZero();
  states[0].alpha.setZero();
  states[0].acceleration.setZero();
  out.state.q = q;
  out.state.dq = dq;
  out.state.mass.setZero();
  out.state.coriolis.setZero();
  out.gravity.setZero();
  for (int index = 1; index < count_; ++index) {
    const auto& b = bodies_[index];
    const auto& parent = states[b.parent];
    auto& s = states[index];
    const Eigen::Vector3d r = parent.pose.linear() * b.origin.translation();
    s.pose = parent.pose * b.origin;
    s.omega = parent.omega;
    s.alpha = parent.alpha;
    s.acceleration =
        parent.acceleration + parent.alpha.cross(r) + parent.omega.cross(parent.omega.cross(r));
    if (b.joint >= 0) {
      const int j = b.joint;
      origins[j] = s.pose.translation();
      axes[j] = s.pose.linear() * b.axis;
      s.omega += axes[j] * dq[j];
      s.alpha += parent.omega.cross(axes[j] * dq[j]);
      s.pose = s.pose * Eigen::AngleAxisd(q[j], b.axis);
    }
    const Eigen::Vector3d offset = s.pose.linear() * b.com;
    const Eigen::Vector3d com = s.pose.translation() + offset;
    Eigen::Matrix<double, 3, 7> jv = Eigen::Matrix<double, 3, 7>::Zero();
    Eigen::Matrix<double, 3, 7> jw = Eigen::Matrix<double, 3, 7>::Zero();
    for (int j = 0; j < 7; ++j)
      if (b.ancestors & (1u << j)) {
        jw.col(j) = axes[j];
        jv.col(j) = axes[j].cross(com - origins[j]);
      }
    const Eigen::Matrix3d inertia = s.pose.linear() * b.inertia * s.pose.linear().transpose();
    out.state.mass.noalias() += b.mass * jv.transpose() * jv;
    out.state.mass.noalias() += jw.transpose() * inertia * jw;
    const Eigen::Vector3d force =
        b.mass * (s.acceleration + s.alpha.cross(offset) + s.omega.cross(s.omega.cross(offset)));
    const Eigen::Vector3d moment = inertia * s.alpha + s.omega.cross(inertia * s.omega);
    out.state.coriolis.noalias() += jv.transpose() * force + jw.transpose() * moment;
    out.gravity.noalias() -= jv.transpose() * (b.mass * Eigen::Vector3d(0, 0, -9.81));
  }
  for (int frame = 0; frame < 9; ++frame) {
    const int body = frame < 7 ? joint_bodies_[frame] : flange_;
    out.state.frame_pose[frame] = states[body].pose;
    if (frame == 8)
      out.state.frame_pose[frame] = out.state.frame_pose[frame] * flange_to_ee;
    auto& jac = out.state.frame_jacobian[frame];
    jac.setZero();
    for (int j = 0; j < 7; ++j)
      if (bodies_[body].ancestors & (1u << j)) {
        jac.col(j).head<3>() =
            axes[j].cross(out.state.frame_pose[frame].translation() - origins[j]);
        jac.col(j).tail<3>() = axes[j];
      }
  }
  return out.state.mass.allFinite() && out.state.coriolis.allFinite() && out.gravity.allFinite();
}
}  // namespace teleop_controllers
