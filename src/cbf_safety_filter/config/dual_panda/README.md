# Shared dual-Panda CBF configuration

`teleop_bringup/config/dual_panda/dual_panda.yaml` loads `cbf.yaml` directly.
The profile uses latest-snapshot recovery and selects `projection` under
`execution.safety_torque_computation_method`. The optional `native_qp` setting uses the
selected QP backend with a verified projection fallback. Restart the launch
after editing the profile. Each owner selects its collision geometry by
`controller_id`. See [snapshot recovery](../../../../docs/cbf/SNAPSHOT_RECOVERY.md)
and the [interactive geometry viewer](../../../../docs/figures/collision_model/viewer.html).

| Controller | Robot IP | WORLD base translation | WORLD base orientation |
|---|---|---|---|
| Left | 192.168.3.101 | [0, 0, 0] m | Identity |
| Right | 192.168.3.109 | [0, -0.750, 0] m | Identity |

Both arms retain their metal fingers plus the confirmed 5 g UMI attachments.
Each arm has 15 collision volumes, including a Hand housing capsule and four
volumes that follow finger opening. The table checks the same cover except the
base and Link 1. Physical table height is 0.127 m, with 0.5 mm clearance. Inter-arm clearance is 10 mm,
reference activation is 15 mm, and peer age/skew limits are 10000 us.
Joint position, velocity and acceleration constraints are enabled and hard.
The joint acceleration limits are [15, 7.5, 10, 12.5, 15, 20, 20] rad/s².
Filter/callback computation budgets remain at the saved 800/800 us.

Each owner uses the last valid peer collision volumes. Snapshots expire after
10 ms. Held arms retain their holding controller and local safety limits.
Active arms handle inter-arm avoidance independently.

Older policies can still be used with their matching historical sphere geometry.
The shipped capsule geometry uses snapshot recovery.

The separate `config/single_panda/cbf.yaml` is the single-arm launch profile and has no inter-arm
section. Single-geometry custom profiles continue to use `geometry.file`.
Geometry files retain the detailed body cover and surveyed transforms; different
transforms are essential even though the two arms share numerical settings.

Controller, mapper, Hand, homing and recording settings remain in the bringup
runtime YAML. The reviewed capsule and sphere cover is the measurement model;
the operator will tune clearance manually against physical trials. The only
configured inter-arm policy is `independent`; older shared and predictive
implementations are no longer available. This path requires its own
timing/hardware qualification.
