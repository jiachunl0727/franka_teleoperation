# Historical hardware commissioning record

This file records earlier commissioning decisions and trial status. As of
2026-11-22, inter-arm CBF configuration supports only independent per-arm
filtering; shared and predictive implementations described in this history
have been removed and are not available for new launches.

**Latest dual preparation:** the operator requests exactly
[combined local checks](../../log/commissioning/2026-11-12-dual-cbf/COMBINED_TRIAL.md)
then [inter-arm avoidance](../../log/commissioning/2026-11-12-dual-cbf/INTER_ARM_TRIAL.md).
At the time, the experimental paired path supported inter-arm rows with valid
stationary startup seeds and peer freshness checks. That shared implementation
has since been removed. This section records historical trial preparation and
does not describe a currently available policy. Older blocked-status notes below
are also historical.

**2026-11-12 update:** the normal single-arm recording is
[reviewed](../../log/commissioning/2026-11-11-hardware-cbf/review/single-arm-20261112T133654Z/RESULTS.md).
Explicit commissioning now also supports two independent per-arm CBF owners,
requiring both owners enabled/confirmed in the same shadow/filtered mode.
Inter-arm hardware constraints remain blocked pending peer-startup and measured
WORLD-geometry preparation. See the
[dual preparation record](../../log/commissioning/2026-11-12-dual-cbf/README.md).
Activation defaults remain off/blocked. Timing diagnosis is deferred at the
operator's request; timing/freshness guards are unchanged.

The 2026-11-11 filtered approach exposed a terminal `invalid_domain` after table
torque intervention. The subsequent [reference and rejection-capture correction](BOUNDARY_REFERENCE_AND_REJECTION_CAPTURE.md)
adds table input blocking, immediate retreat after blocked overtravel and an early
terminal snapshot. Use the new code and preserve the complete launch log before
another reviewed trial. A passing frozen replay does not reconstruct the missing
terminal state or establish a physical boundary hold. The existing operator-run
session launcher already saves the terminal JSON automatically through `tee`.

The software now supports an explicit single-arm commissioning opt-in. Defaults
remain blocked; this is experimental commissioning, not hardware qualification.
Inter-arm hardware CBF remains blocked. No robot is started by the software tests.
See [software verification](COMMISSIONING_SOFTWARE_VERIFICATION.md) for the checks performed.

For the confirmed 2026-11-11 installation, the
[prepared session and exact launch commands](../../log/commissioning/2026-11-11-hardware-cbf/README.md)
provide arm-only baseline/shadow copies and later filtered stages, with Hand and
automatic homing off and continuous recording. Its full CBF copy includes
self-collision and singularity as well as joint limits and the table. The
recorded replay passes, but a forced-solver mock timing stop is retained in that
session; new hardware shadow timing must be reviewed before filtered operation.

The current URDF model's successful replay does not qualify physical filtered
intervention. Its inherited inertia values have no established physical calibration
in the inspected sources. The custom printed fingers now have a mounted,
full-opening collision cover and estimated 5 g attachment dynamics; verify the
placement and deformation assumptions in the
[finger/replay report](UMI_FINGERS_AND_OFFLINE_AVOIDANCE.md). Review the
[model evidence and derivation](URDF_MODEL_JUSTIFICATION.md) before advancing
beyond shadow; repeat the ordinary small-motion baseline with the new runtime.

1. **Prepare the installation.** Check tool/payload, table height, base transform
   and arm/Hand geometry. Confirm small translations and rotations, including
   wrist motion, behave normally with CBF disabled. Use generous clearance and
   an operator at the stop control. Shadow mode provides **no CBF protection**.
   Table height is the physical tabletop coordinate in WORLD, not a raised TCP
   plane. Hand/finger sizes belong to geometry and clearance is applied separately.
   The confirmed single-arm installation uses 0.127 m and 0.005 m clearance;
   adding the previous 0.063 m finger allowance again counted the fingers twice.
2. **Record a short shadow trial.** Copy the single-arm YAML and set:

   ```yaml
   cbf:
     enabled: true
     config_file: package://cbf_safety_filter/config/single_panda/cbf.yaml
     hardware_mode: shadow
   recording:
     mode: continuous
     include_default_topics: true
   ```

   Pass the copied YAML as `runtime_config`. Set `cbf.enabled: false` to bypass CBF.
   The CBF YAML selects synchronous or asynchronous solving through `execution.mode`.
   Synchronous mode is currently selected and computes the current-cycle QP in
   the callback. Hardware requires timing/residual diagnostics and a callback
   budget no greater than 800 µs. Native solve time is part of that synchronous
   budget; async callback timings alone do not qualify synchronous execution.
   Start clear of all boundaries; perform small, slow motions. Stop recording
   cleanly and retain the complete bag directory, launch log and exact configs.
3. **Review before applying corrections.** Inspect callback times, model time,
   callback intervals, CBF statuses, proposed corrections and dropped samples.
   Rejected proposals in shadow mode are recorded while the nominal controller
   continues. Investigate unexpected rejection or large correction first.
   Shadow mode must not be used to test collision or singularity prevention.
4. **Run a short filtered trial after review.** Change only
   `hardware_mode: filtered`, restart, and keep continuous recording. With
   generous configured margins, check connected holding, feasible sliding and
   retreat. Enable only the constraint families whose geometry/settings have
   been checked: self/inter-arm/singularity remain disabled in the example CBF
   file unless explicitly changed. Do not approach contact or an actual
   singularity. An unexplained wrist restriction, torque jump or oscillation
   ends the trial for inspection.

Send the bag directory, complete launch log, exact runtime/CBF/geometry files,
code revision and a brief description of attempted motions for review. An opt-in
is the operator's explicit commissioning choice; the software does not certify
that a trial is safe or automatically approve expansion to two arms.

`cbf_status` records `output_mode` (`shadow`/`filtered`), proposal status,
`actual_command_verified`, worker latency/age, residuals, holding and queue drops.
In shadow mode, `accepted` refers to the computed proposal, **not** to the torque
issued by the nominal controller; that issued command may violate CBF rows.
No CBF reference projection or holding torque is applied in shadow mode.
Invalid CBF domains/proposals alone do not interrupt shadow control. Existing
controller/input checks and the callback deadline remain enforced in both modes.
Filtered mode still stops if no fresh verified command exists; no automatic
restart or retreat is added.

`nominal_command_feasible` independently inspects the actuator-clipped nominal
against current original rows; `nominal_domain_valid`, `nominal_max_row_violation`
and `nominal_worst_family` distinguish domain/row failures from unavailable
proposals. `current_box_infeasible_rows` counts original rows impossible for
every torque in the current actuator interval; zero does not prove joint
feasibility of all rows. `proposal_residuals_valid` is false when the displayed
proposal residual slots are not backed by an accepted proposal. These are
observations, not command authority or physical guarantees. In particular,
`actual_command_verified:false` alone does not prove the nominal command violated
a row. Family numbers follow `types.hpp` (position=0, velocity=1, table=3).
See [shadow diagnosis](SHADOW_FAILURE_DIAGNOSIS.md).
For replayable equations and candidate checks, use the saved historical snapshots
and the offline replay procedure in [failure diagnosis](FAILURE_CAPTURE.md).

Hardware `controller_callback_us` starts at callback entry before model feedback
acquisition and ends at the final diagnostic handoff. Thread CPU time uses that
same entry point. `model_acquisition_us` covers feedback acquisition and copying;
`observed_period_us` measures callback entry intervals (first sample is zero).
Tiny final return/copy work and FCI network transport are outside that recorded
interval. Hardware wake-up lateness is not measured and is marked invalid.
The 800 µs check includes model acquisition; it is an overrun detector, not a
scheduler guarantee. Missed intervals, missing cycles and queue drops must also
be reviewed. Recording can itself add non-RT load: verify with recording enabled.

The prior [mock timing report](CALLBACK_BUDGET_REPORT.md) and its limits remain
relevant. Physical model agreement, stopping response, active-constraint timing,
tracking continuity and geometry require commissioning evidence; holding a
Cartesian pose does not guarantee collision-free braking.
