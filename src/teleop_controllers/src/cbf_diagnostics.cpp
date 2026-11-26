#include "cbf_diagnostics.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_controllers {
namespace {
const char* family_name(cbf_safety_filter::Family f) {
  using F = cbf_safety_filter::Family;
  switch (f) {
    case F::position:
      return "joint_position";
    case F::velocity:
      return "joint_velocity";
    case F::acceleration:
      return "joint_acceleration";
    case F::table:
      return "table";
    case F::obstacle:
      return "obstacle";
    case F::self_collision:
      return "self_collision";
    case F::inter_arm:
      return "inter_arm";
    case F::singularity:
      return "singularity";
    case F::peer_motion:
      return "peer_motion";
  }
  return "unknown";
}
}  // namespace
void write_fci_timing(std::ostream& out, const FciTiming& t) {
  if (!t.valid) {
    out << "null";
    return;
  }
  out << "{\"state_id\":" << t.state_id << ",\"received_states\":" << t.received_states
      << ",\"waited\":" << (t.waited ? "true" : "false")
      << ",\"update_wall_us\":" << t.update_wall_us << ",\"update_cpu_us\":" << t.update_cpu_us
      << ",\"receive_wall_us\":" << t.receive_wall_us
      << ",\"post_receive_us\":" << t.post_receive_us << ",\"to_callback_us\":" << t.to_callback_us
      << '}';
}
void write_cbf_terminal_snapshot(std::ostream& out, const CbfTerminalSnapshot& s,
                                 const std::string& fingerprint) {
  out.precision(17);
  const auto number = [&](double v) {
    if (std::isfinite(v))
      out << v;
    else
      out << "null";
  };
  const auto vector = [&](const auto& v) {
    out << '[';
    for (int j = 0; j < v.size(); ++j) {
      if (j)
        out << ',';
      number(v[j]);
    }
    out << ']';
  };
  out << "{\"schema\":\"cbf_failure_v1\",\"terminal\":true,\"accepted\":false,"
         "\"command_issued\":" << (s.protective_command_issued ? "true" : "false")
      << ",\"protective_stop\":" << (s.protective_command_issued ? "true" : "false")
      << ",\"control_terminated\":" << (s.control_terminated ? "true" : "false")
      << ",\"status\":\""
      << cbf_safety_filter::name(s.status) << "\",\"stage\":\"" << s.stage
      << "\",\"cycle\":" << s.cycle << ",\"state_time_pc_monotonic_ns\":" << s.state_ns
      << ",\"previous_cycle\":" << s.previous_cycle
      << ",\"previous_command_issue_time_pc_monotonic_ns\":" << s.previous_issue_ns
      << ",\"protective_command\":";
  if (s.protective_command_issued)
    vector(s.protective_command);
  else
    out << "null";
  out << ",\"model_fnv1a64\":\"" << fingerprint << '"' << ",\"backend\":\""
      << cbf_safety_filter::name(s.result.backend) << "\",\"rows\":" << s.result.rows
      << ",\"solver_iterations\":" << s.result.iterations
      << ",\"requested_qp_status\":\"" << cbf_safety_filter::name(s.result.requested_qp_status) << '"'
      << ",\"recovery_qp_status\":\"" << cbf_safety_filter::name(s.result.recovery_qp_status) << '"'
      << ",\"recovery_projection_fallback\":" << (s.result.recovery_projection_fallback ? "true" : "false");
  for (const auto& field : {std::pair<const char*, double>{"period_seconds", s.period},
                            {"observed_period_us", s.observed_period_us},
                            {"callback_us", s.callback_us},
                            {"callback_cpu_us", s.callback_cpu_us},
                            {"model_acquisition_us", s.model_acquisition_us}}) {
    out << ",\"" << field.first << "\":";
    number(field.second);
  }
  out << ",\"fci_timing\":";
  write_fci_timing(out, s.fci);
  out << ",\"filter_timing_valid\":" << (s.filter_timing_valid ? "true" : "false")
      << ",\"filter_us\":";
  if (s.filter_timing_valid)
    number(s.result.total_us);
  else
    out << "null";
  out << ",\"callback_cpu\":" << s.callback_cpu << ",\"filter_phases_us\":";
  if (!s.filter_timing_valid) {
    out << "null";
  } else {
    out << '{';
    bool comma = false;
    double accounted = 0.;
    for (const auto& field : {std::pair<const char*, double>{"model", s.result.model_us},
                              {"kinematics", s.result.kinematics_us},
                              {"constraints", s.result.constraints_us},
                              {"assembly", s.result.assembly_us},
                              {"update", s.result.update_us},
                              {"solver", s.result.solve_us},
                              {"verification", s.result.verify_us}}) {
      if (comma)
        out << ',';
      comma = true;
      out << '"' << field.first << "\":";
      number(field.second);
      accounted += field.second;
    }
    out << ",\"transaction_other\":";
    number(std::max(0., s.result.total_us - accounted));
    out << '}';
  }
  for (const auto& field :
       {std::pair<const char*, double>{"max_hard_violation", s.result.max_hard_violation}}) {
    out << ",\"" << field.first << "\":";
    if (s.result.accepted())
      number(field.second);
    else
      out << "null";
  }
  out << ",\"q\":";
  vector(s.model.q);
  out << ",\"dq\":";
  vector(s.model.dq);
  out << ",\"coriolis\":";
  vector(s.model.coriolis);
  out << ",\"incoming_robot_desired_torque\":";
  vector(s.model.previous_torque);
  out << ",\"measured_torque\":";
  vector(s.measured_torque);
  out << ",\"gravity\":";
  vector(s.gravity);
  out << ",\"control_request_ns\":" << s.control_request_ns
      << ",\"own_startup_seed_ns\":" << s.own_startup_seed_ns
      << ",\"peer_startup_seed_ns\":" << s.peer_startup_seed_ns
      << ",\"startup_observation_count\":" << s.startup_observation_count
      << ",\"last_startup_observation_ns\":" << s.last_startup_observation_ns;
  out << ",\"peer\":{\"valid\":" << (s.peer.valid ? "true" : "false")
      << ",\"state_ns\":" << s.peer.timestamp_ns << ",\"count\":" << s.peer.count
      << ",\"speed_limit\":";
  number(s.peer.speed_limit);
  out << ",\"acceleration_limit\":";
  number(s.peer.acceleration_limit);
  out << ",\"world_spheres\":[";
  for (int i = 0; i < s.peer.count && i < cbf_safety_filter::kMaxPoints; ++i) {
    if (i)
      out << ',';
    out << "{\"position\":";
    vector(s.peer.spheres[i].position);
    out << ",\"velocity\":";
    vector(s.peer.spheres[i].velocity);
    out << ",\"radius\":";
    number(s.peer.spheres[i].radius);
    out << ",\"span\":";
    vector(s.peer.spheres[i].span);
    out << '}';
  }
  out << "]}";
  out << ",\"mass_matrix_rows\":[";
  for (int j = 0; j < 7; ++j) {
    if (j)
      out << ',';
    vector(s.model.mass.row(j));
  }
  out << "],\"torque_lower\":";
  vector(s.bounds.lower);
  out << ",\"torque_upper\":";
  vector(s.bounds.upper);
  out << ",\"previous_command_valid\":" << (s.previous_command_valid ? "true" : "false")
      << ",\"last_issued_torque\":";
  if (s.previous_command_valid)
    vector(s.last_command);
  else
    out << "null";
  out << ",\"nominal_torque\":";
  if (s.nominal_available)
    vector(s.nominal);
  else
    out << "null";
  out << ",\"requested_pose_valid\":" << (s.requested_pose_valid ? "true" : "false");
  out << ",\"requested_position\":";
  if (s.requested_pose_valid)
    vector(s.requested_position);
  else
    out << "null";
  out << ",\"requested_orientation_xyzw\":";
  if (s.requested_pose_valid)
    vector(s.requested_orientation.coeffs());
  else
    out << "null";
  out << ",\"reference_position\":";
  if (s.nominal_available)
    vector(s.reference_position);
  else
    out << "null";
  out << ",\"reference_orientation_xyzw\":";
  if (s.nominal_available)
    vector(s.reference_orientation.coeffs());
  else
    out << "null";
  out << ",\"proposal_residuals_valid\":" << (s.result.accepted() ? "true" : "false")
      << ",\"proposal_status\":\"" << cbf_safety_filter::name(s.result.status)
      << "\",\"candidate_torque\":";
  if (s.nominal_available && s.result.accepted())
    vector(s.result.torque);
  else
    out << "null";
  out << ",\"failure\":";
  if (s.failure.present) {
    const auto& r = s.failure.row;
    out << "{\"family\":\"" << family_name(r.family) << "\",\"family_id\":" << int(r.family)
        << ",\"row_index\":" << s.failure.row_index << ",\"h\":";
    number(r.h);
    out << ",\"psi\":";
    number(r.psi);
    out << ",\"bound\":";
    number(r.bound);
    out << ",\"coefficients\":";
    vector(r.g);
    out << '}';
  } else
    out << "null";
  out << '}';
}
}  // namespace teleop_controllers
