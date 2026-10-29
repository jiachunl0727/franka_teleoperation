# Saved model regression fixtures

These 23 `CBF_MODEL_V1` snapshots came from first-hold diagnostics in the
2026-10-28 18:05 CET trial (17:05 UTC in the log directory; controller PID 5479). Each contains the arm index,
cycle/period, q, dq, Coriolis torque, previous desired torque, nominal torque,
mass matrix, and nine frame transforms/Jacobians. Matrices are column-major.

They reproduce strong real-model inertia coupling absent from the earlier
diagonal and lightly coupled software fixtures. Acceptance tolerance was 0.001;
20 logged holds exhausted local repair at residuals 0.00100006–0.001673,
and three rejected a worker result. Several previous torque snapshots already
imply acceleration at or slightly beyond the ±7 rad/s² configured bound.

Tests replay each model independently with the original nominal and perturbed
targets. They do not reconstruct the exact cached worker correction, prior
derivative history, packet timing or subsequent physical trajectory. Separate
feedback tests verify motion on synthetic plants: the original step cases and
sampled 0.5 s trajectories with this measured inertia. Abrupt steps in the latter
simplified plant are not a passed tracking guarantee. These fixtures contain no
robot connection or operator input recording and never operate hardware.
