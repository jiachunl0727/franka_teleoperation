std_msgs/Header header
bool hardware_backend
bool solver_succeeded
bool fallback_active
bool fatal_input_fault
string solver_status
string stop_reason
uint64 control_cycle
uint64[2] source_time_pc_monotonic_ns
uint64 state_skew_ns
uint32 consecutive_failures
float64 filter_time_seconds
float64 solver_time_seconds
float64 maximum_constraint_violation
float64 minimum_table_clearance
float64 minimum_self_clearance
float64 minimum_inter_arm_clearance
float64 intervention_norm
int32 active_constraints
int32 inter_arm_constraints
float64[7] safe_torque

# Dimensionless sqrt(det(Js Js^T)); zero when singularity rows are disabled.
float64[2] manipulability
uint32 singularity_constraints
bool singularity_state_invalid
bool fatal_solver_fault

# Asynchronous runtime diagnostics. Worker age is from its source model timestamp.
string execution
string worker_status
string fallback_reason
float64 worker_age_ms
bool worker_result_usable
bool peer_state_usable
bool braking_hold_active
bool hold_settled
bool model_reused
bool mandatory_table_guard_enabled
uint64[2] worker_source_cycle
