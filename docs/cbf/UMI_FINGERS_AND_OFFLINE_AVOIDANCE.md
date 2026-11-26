# Installed UMI fingers and offline avoidance verification

Date: 2026-11-09. All operations described below are offline or mock. No robot,
Hand or headset was operated, and no hardware activation switch was enabled.

## Installed model

The previous model contained stock fingers only. The installed configuration now
has an augmented [panda_umi.urdf](../../src/teleop_controllers/config/panda_umi.urdf)
and [geometry_panda_umi.yaml](../../src/cbf_safety_filter/config/single_panda/geometry_panda_umi.yaml).
The exact upstream OSCBF asset remains available as `panda_oscbf.urdf`.

[Interactive collision viewer](../figures/collision_model/viewer.html)
displays the current dual-arm geometry. Regenerate it with
`python3 scripts/visualize_umi_model.py` after editing collision geometry or
clearances.

The operator confirmed that both stock metal fingers remain, each printed
attachment including its mounting screw weighs approximately **5 g**, the mount
is the metal finger's **gripping face**, and the tip extends away from the Hand,
parallel to the original finger, approximately **72 mm from screw to tip**.

The single-arm YAML and both arms in the dual YAML use `panda_umi.urdf` for
dynamics. Their CBF profiles select geometry that includes the UMI attachments.
Generic standalone controller defaults retain the stock asset. Select
`panda_umi.urdf` in a standalone profile for a Hand fitted with these attachments.

### Kinematics and mounting

Two fixed attachment links, `panda_umi_leftfinger` and
`panda_umi_rightfinger`, are children of the retained stock finger links. Each
has an STL visual, a local collision box and an inertial block. The existing
arm/EE frames and control TCP are unchanged: adding collision geometry does
not silently move the teleoperation control point to the printed tips.

The original STL is preserved at
[config/meshes/umi_v1.stl](../../src/teleop_controllers/config/meshes/umi_v1.stl),
with SHA-256
`8a042fc254b988c7af7e025dddf79361ea7edec7f7eed315f7ee1de94081ff19`.
Its millimetre-to-metre scale is explicit. It is a watertight 1,906-triangle
mesh, approximately 20 × 12.9 × 78.18 mm.

The mounting transform uses the stock screw/face dimensions from Franka's
fingertip-interface drawing, checked against the pinned stock finger mesh.
The screw is at approximately `(0, 9.4, 45)` mm in the stock finger frame;
45 mm includes the mesh's 4 mm mounting offset from the drawing's datum.
The STL screw axis runs along its X direction at approximately
`Y=6.45, Z=69.9323` mm. Its tip is near STL Z=0, so **STL −Z**, rather than
STL +Z, must point away from the Hand. [Franka Hand manual, figure 5.6](https://download.franka.de/documents/Product%20Manual%20Franka%20Hand_R50010_1.1_EN.pdf).

For the left finger, the mesh-to-stock transform in metres is

\[
R=\begin{bmatrix}0&1&0\\1&0&0\\0&0&-1\end{bmatrix},\qquad
t=\begin{bmatrix}-0.006450\\-0.01441955\\0.11493233\end{bmatrix}.
\]

The right attachment is its 180-degree mirror about the Hand Z axis. The
derived rigid tip is about 61 mm beyond the stock fingertip, consistent with
the earlier approximate 63 mm extension allowance. A **3 mm mounting allowance**
covers the difference between the CAD screw-to-tip distance and the operator's
approximate 72 mm measurement. This is an explicit installation approximation,
not metrology or a bound on arbitrary soft deformation. The measured-pose/STL
check of the contact recording places the rigid tip about 1.6 mm above the
physical table at the final state, consistent with that approximation.

The transform, mass, opening range and allowance are configurable in
[umi_fingers.yaml](../../src/teleop_controllers/config/umi_fingers.yaml).
The [mounting plot](../../log/commissioning/2026-11-09-fingers-avoidance/umi-mount.png)
shows both meshes and their screw locations. Verify this placement against
the actual installation before treating its clearance as measured truth.

### Dynamics

The added mass is **0.010 kg total**, in addition to the retained stock model.
Centre of mass and rotational inertia come from signed tetrahedral volume
integrals of the watertight STL, assuming homogeneous density and scaling the
result to the measured 5 g per attachment. No inherited `0.1` tensor is assigned
to the new prints. Their tensor diagonals about their own CoM are approximately

\[
I_C=\begin{bmatrix}
1.94277\times10^{-6}&0&-5.75728\times10^{-8}\\
0&2.01833\times10^{-6}&0\\
-5.75728\times10^{-8}&0&2.16128\times10^{-7}
\end{bmatrix}\ \mathrm{kg\,m^2}.
\]

Small near-zero products from mesh roundoff are retained in the generated file.
The STL-frame CoM is approximately `(17.1801, 6.4500, 50.2705)` mm. Native
runtime dynamics include each attachment's rotated tensor and parallel-axis
effects in M/c/g. Both nominal laws and CBF receive those same augmented values.

The actual print and screw need not have uniform density, so these CoM/tensor
values are estimates. The arm remains a seven-joint fixed-attachment model:
its dynamics use the configured **closed-reference opening**, not live Hand
width or finger acceleration. The collision envelope independently covers all
openings. Desk compensation/load metadata is not changed by editing this URDF;
the physical controller's load configuration is a separate installation setting.

These additions do not resolve the underlying physical-calibration limits of
the inherited OSCBF inertias described in [the model justification](URDF_MODEL_JUSTIFICATION.md).

### Collision envelope

The runtime table route uses capsules; self/inter-arm routes use sphere covers,
and obstacle rows use explicitly paired points. It does not consume URDF visuals
or collision tags directly.

The generator encloses both transformed prints over **0–40 mm travel per
finger**, including every intermediate opening, in a Hand-frame box, padded by
3 mm. That box is partitioned into 12 cells. Each cell's circumsphere contains
the entire cell, so the proof covers triangle interiors and continuous opening
travel as well as mesh vertices. These spheres are exported as:

- 12 additional zero-length table capsules;
- 12 additional obstacle points, with pairs to existing configured obstacles;
- 12 additional body spheres for collision routes.

The result has **24 table capsules, 13 obstacle points and 55 body spheres**,
within the existing capacities. The printed parts use group 10. They are
excluded only from their mounting Hand group 9, so they do not inherit the
stock Hand's broader arm-link exclusions. Existing stock metal/Hand covers stay.

The physical table is still **0.127 m**, with separate **0.005 m clearance**.
The 3 mm mounting allowance is in attachment geometry; finger length is not
added to table height.

Regenerate/check both models from the same source:

```bash
python3 scripts/generate_umi_finger_model.py
python3 scripts/generate_umi_finger_model.py --check
```

The generator needs NumPy and PyYAML, with no robot connection. The generated
JSON alongside the URDF records volume, CoM, tensors, transforms and box bounds.

## What to verify from original recorded samples

The new [replay script](../../scripts/replay_avoidance.py) opens the original
SQLite bag read-only, checks database integrity, deserializes the actual
`TrackingSample` messages and preserves source hashes. It includes inactive,
holding and active states; reports missing cycles/dropped samples; and rejects
missing incoming torque or incoherent sample order.

The installed `avoidance_replay` executable reconstructs the current URDF model
at every recorded q/dq and verifies reconstructed pose against measured pose.
It rebuilds magnitude/slew intervals from the actual incoming `tau_J_d`, clips
the original nominal torque and compares that reconstruction to the recorded
issued torque. It then runs four explicit offline profiles:

1. Configured enabled families.
2. Table only, to isolate the table route.
3. Singularity only, explicitly enabled in the offline copy.
4. Current joint/table families plus singularity.

All retain production torque limits, barrier parameters, solver/slack policy,
freshness guards and deadlines. Historical asynchronous traces are treated as
current frozen problems; worker scheduling is not reproduced. No production
activation setting is edited.

### Table checks

For every capsule endpoint,

\[
h=z-z_{\rm table}-r-d,\quad
\dot h=J_z\dot q,\quad \psi=\dot h+\alpha_1h.
\]

The geometric/approach domain requires h and psi nonnegative. For an accepted
torque, the second-order condition is

\[
J_zM^{-1}(u-c)+\dot J_z\dot q+
(\alpha_1+\alpha_2)J_z\dot q+\alpha_1\alpha_2h\ge0.
\]

The replay checks the actual native rows and independently rechecks the exact
actuator-clipped output. It also detects individual rows that no torque in the
current interval can satisfy. A separate analysis transforms the raw STL through
the **recorded measured pose**, independently of native FK and capsule endpoint
calculations, and confirms the conservative cover lies below the exact rigid
printed-finger sweep.

### Singularity checks

The complete six-dimensional Jacobian is normalized with the configured
characteristic length l=0.3 m:

\[
J_n=\operatorname{diag}(1/l,1/l,1/l,1,1,1)J,\qquad G=J_nJ_n^T.
\]

The runtime's smooth measure is

\[
\rho=1/\operatorname{tr}(G^{-1})\le\sigma_{\min}(J_n)^2.
\]

It enforces \(h_s=\rho-0.025^2\), rather than using SVD sigma_min directly as
the barrier value. Both values are reported to prevent confusing them. Its
second-order row is

\[
\nabla\rho^TM^{-1}(u-c)+\rho_{\rm bias}+
(\alpha_1+\alpha_2)\nabla\rho^T\dot q+\alpha_1\alpha_2h_s\ge0.
\]

Analytic gradients and directional acceleration biases are checked against
central differences of independently evaluated FK/SVD on representative raw
recorded states. The maximum observed discrepancies are approximately
3e-11 for the singularity gradient, 3e-8 for its directional bias and 9e-8 for
the table endpoint bias across both recordings. No cross-sample acceleration
prediction fit is needed for these derivative checks.

## Results

### Free-space recording

Original bag: `quest_20261105T150001Z_89a8d8e9`.

- 46,418 total states, including 30,445 active teleoperation states.
- Zero missing control cycles, reported monitor drops or clipping mismatches.
- Minimum conservative table margin: **121.793 mm**, after all allowances.
- Minimum normalized SVD sigma_min: **0.390545**.
- Minimum smooth lower-bound sqrt(rho): **0.310117**, versus threshold 0.025.
- All 46,418 states pass each replay profile, including combined table and
  singularity; all 30,445 active outputs are verified.
- No additional CBF torque correction is needed in this free-space recording.

This confirms compatibility in that region. Its large margins mean the
recording does not exercise real singularity-boundary intervention.
[Full summary](../../log/commissioning/2026-11-09-fingers-avoidance/free-space/summary.json).

### Intentional-contact recording

Original bag: `quest_20261105T132035Z_3ef3e59a`.

There are 45,777 states, including 30,343 active states, with no missing cycles,
reported monitor drops or clipping mismatches. Replaying with only the old
stock geometry reports **+49.701 mm** minimum table margin and no table
intervention. The augmented geometry reports **−10.104 mm** minimum margin.
The independent rigid-STL calculation gives 1.633 mm above the physical plane,
or **−3.367 mm relative to the separate 5 mm clearance**, before mounting/cover
allowances. This is consistent with the approximate installation and reported
contact; it does not timestamp contact onset or measure soft deformation.

For the **table-only** profile:

| Event | Cycle | Table margin | Interpretation |
| --- | ---: | ---: | --- |
| First verified correction | 39,366 | +23.279 mm | Nominal torque begins violating a table row |
| First rejection | 39,383 | +21.973 mm | Current frozen actuator interval cannot satisfy the table constraints |
| First approach-domain failure | 39,501 | +10.691 mm | psi becomes negative before geometric clearance is exhausted |
| First negative geometric margin | 39,616 | Slightly below zero | Conservative cover enters its clearance boundary |
| Minimum margin | 45,918 | −10.104 mm | Final recorded state |

The first table correction is 6.588 s before the last recorded state. This is
**not** a measured time-to-contact because onset is not timestamped.

The table-only replay verifies 40,111 states, including **138 corrected
commands**, rejects 318 as infeasible and rejects 5,348 for invalid domain.
The combined profile verifies 40,105 states, including 142 corrections; it also
has 6 positive-slack rejections from the additional current joint constraints.
Singularity-only accepts every recorded state; minimum smooth sqrt(rho) is
0.317123, far from its threshold.

These are appropriate detections of an unsafe recorded approach, but they do
**not** prove successful physical avoidance. Every later replayed state still
comes from the original unfiltered trajectory, even after a hypothetical
correction or stop. The infeasible intervals show a loss of available modeled
braking authority along that history. A closed-loop rollout would be needed to
evaluate the changed future trajectory, and hardware stopping response still
needs staged commissioning evidence. We did not weaken rows or accept unsafe
commands to obtain a higher rate.
[Full summary](../../log/commissioning/2026-11-09-fingers-avoidance/contact/summary.json),
[old-geometry control](../../log/commissioning/2026-11-09-fingers-avoidance/contact-stock-geometry/summary.json).

![Contact replay comparison](../../log/commissioning/2026-11-09-fingers-avoidance/contact-avoidance.png)

### Deliberate offline boundary challenges

`--boundary-probes` additionally runs two clearly labeled synthetic challenges.
A real recorded configuration seeds a deterministic search for a state just
inside the **actual configured** table or singularity boundary. Velocity is
zero, previous torque is a synthetic holding history, and magnitude/slew caps
remain unchanged. A deliberately violating nominal is chosen within that box.
These are not original bag samples or a physical robot simulation.

- At table h=0.1 mm, the filter corrects the violating nominal; outgoing row
  residual is approximately −1e-16.
- At singularity h=1e-7, it also corrects the violating nominal; outgoing row
  residual is approximately +5e-17, within the unchanged 1e-6 tolerance.

Both exact actuator-clipped outputs pass the production verifier. These tests
exercise active intervention and fail if either route silently passes the
violating nominal. They do not establish invariant-set preservation under
discrete sampling, model error or physical braking.

## Repeat the checks before a hardware trial

Build/source the usual ROS workspace, then run inside the ROS container at `/ws`:

```bash
python3 scripts/generate_umi_finger_model.py --check
python3 scripts/replay_avoidance.py \
  --bag log/commissioning/2026-11-05-cbf-forensics/bags/quest_20261105T150001Z_89a8d8e9 \
  --cbf-config src/cbf_safety_filter/config/single_panda/cbf.yaml \
  --urdf src/teleop_controllers/config/panda_umi.urdf \
  --boundary-probes \
  --output log/commissioning/avoidance-free-space-check
```

Repeat with the contact bag and a different output directory. Retain each
summary, raw native JSONL, derivative-check log and input/model/config hashes.
Distinguish domain failure, infeasibility, positive-slack rejection and timing
rejection; an aggregate acceptance rate hides these differences.

The geometry/dynamics generator, original OSCBF reference checks, augmented
kinematics/inertia checks and controller/launch regressions pass: **248 unique
cases**, with no failures or skips. An independent PyBullet calculation at
47 recorded states agrees with native augmented M/c/g to at most
2.7e-15 / 1.6e-14 / 1.6e-14 absolute error. This checks calculation consistency
for the supplied model, not its physical calibration.
[Independent-engine results](../../log/commissioning/2026-11-09-fingers-avoidance/independent-engine-check.json).
Full mock
callback checks with augmented geometry and singularity enabled cover both
nominal laws, shadow/filtered modes and forced native QP solves. Their largest
callback/filter times are **553.5/518.673 µs**, below the unchanged 800/600 µs
limits over **13,495 callbacks**, with no measured overruns or dropped diagnostics. Observed mock timing
does not qualify hardware scheduling.

Production CBF remains disabled in the launch configurations; singularity
remains disabled in the example CBF file. The replay enables it only in explicit
offline copies. The next physical stage remains the single-arm baseline/shadow
procedure in [HARDWARE_COMMISSIONING.md](HARDWARE_COMMISSIONING.md), including
checking actual mounting and recording with the new runtime. Offline acceptance
is evidence about the selected model; it is not a release of filtered hardware
protection or dual-arm hardware CBF.
