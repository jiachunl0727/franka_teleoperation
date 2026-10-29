// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once

#include <istream>
#include <ostream>

#include "teleop_controllers/robot_model.hpp"

namespace teleop_controllers {
// Historical column-major format, independent of any execution architecture.
struct ModelSnapshot {
  RobotModelState state;
  Vector7d nominal{Vector7d::Zero()};
  int arm{0};
  uint64_t cycle{0};
  double period{.001};
};
inline bool write_model_snapshot(std::ostream& stream, const ModelSnapshot& snapshot) {
  stream.precision(17);
  stream << "ROBOT_MODEL_V1 " << snapshot.arm << ' ' << snapshot.cycle << ' ' << snapshot.period
         << '\n';
  const auto write = [&](const auto& values) {
    for (Eigen::Index k = 0; k < values.size(); ++k)
      stream << values.data()[k] << ' ';
    stream << '\n';
  };
  write(snapshot.state.q);
  write(snapshot.state.dq);
  write(snapshot.state.coriolis);
  write(snapshot.state.previous_torque);
  write(snapshot.nominal);
  write(snapshot.state.mass);
  for (int frame = 0; frame < kModelFrames; ++frame) {
    write(snapshot.state.frame_pose[frame].matrix());
    write(snapshot.state.frame_jacobian[frame]);
  }
  return bool(stream);
}
inline bool read_model_snapshot(std::istream& stream, ModelSnapshot& snapshot) {
  ModelSnapshot parsed;
  std::string version;
  stream >> version >> parsed.arm >> parsed.cycle >> parsed.period;
  if (!stream || (version != "ROBOT_MODEL_V1" && version != "CBF_MODEL_V1"))
    return false;
  const auto read = [&](auto& values) {
    for (Eigen::Index k = 0; k < values.size(); ++k)
      stream >> values.data()[k];
  };
  read(parsed.state.q);
  read(parsed.state.dq);
  read(parsed.state.coriolis);
  read(parsed.state.previous_torque);
  read(parsed.nominal);
  read(parsed.state.mass);
  for (int frame = 0; frame < kModelFrames; ++frame) {
    read(parsed.state.frame_pose[frame].matrix());
    read(parsed.state.frame_jacobian[frame]);
  }
  if (!stream)
    return false;
  snapshot = parsed;
  return true;
}
}  // namespace teleop_controllers
