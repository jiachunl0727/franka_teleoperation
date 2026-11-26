# OSCBF: public evidence about Franka dynamics mismatch

**2026-11-06 follow-up:** the pinned OSCBF dynamics have now been executed
offline against the recordings. See [exact-model replay](OSCBF_MODEL_REPLAY.md)
for the 32-capture comparison and full 30,445-active-sample results. The public
source review below retains its original 2026-11-05 scope.

Reviewed on **2026-11-05 (Europe/Berlin)**. Research question: do the OSCBF
authors publicly report a large torque-to-acceleration prediction mismatch on
Franka, and, if so, how did they address it? This is a source review, not a
hardware experiment or a validation of this repository's actuation model.

## Finding

**No inspected public source establishes that the authors encountered or fixed
the specific mismatch reported locally: predicted joint accelerations of hundreds
of rad/s² while observed acceleration is around 0.4 rad/s².** The public record
does contain two relevant centrifugal/Coriolis changes, described below. Neither
identifies a Franka acceleration-scale discrepancy or an apparent motor-inertia
correction. This is a bounded negative finding, not proof that the authors never
encountered such a problem privately.

The strongest direct statement is the paper's hardware section: the authors
report “effectively zero mismatch between the hardware data and simulation,”
apart from about 50 ms of teleoperation-interface latency. That is a claim about
their demonstrated behavior; it does not report a measured residual between
`M⁻¹(τ − c − g)` and joint acceleration. [Paper v3, §V.E, p.7](https://arxiv.org/pdf/2503.06736v3#page=7)

## What the authors explicitly report

### Hardware demonstration and limitations of that evidence

The paper describes four Franka Panda experiments with a 1 kHz torque interface,
including containment, collision and singularity avoidance. Reported boundary
approach is within 6 mm. Its dynamics use `M qdd + c + g = Γ`; this equation
alone supplies no evidence of plant-model accuracy. [Paper v3, §III.B and §V.E,
pp.3,7–8](https://arxiv.org/pdf/2503.06736v3#page=7)

The project page links hardware experiment videos and contrasts torque-limited
dynamic control with reduced-order velocity control. It does not explain a
Franka-specific acceleration-model correction. [Project page, “Hardware
Experiments” and dynamic-model comparison](https://stanfordasl.github.io/oscbf/)

In a January 29, 2026 issue reply, Daniel Morton points users to
`oscbf_hardware_ws`: a lightweight libfranka C++ wrapper communicates with a
JIT-compiled JAX Python controller. He reports reliable 1 kHz operation on his
laptop. This establishes that a first-party hardware implementation is publicly
available; execution frequency does not establish acceleration-prediction
accuracy. [OSCBF issue #2, author reply](https://github.com/StanfordASL/oscbf/issues/2#issuecomment-3815485996)

### March 31, 2025: public-release compensation bug

Commit `88ae88f708f9308ed6fa8e6e16690012f127c7c8` identifies a bug introduced
during public-release cleanup: centrifugal/Coriolis forces were not accounted
for properly. The author distinguishes it from the paper results, which used a
different repository. The patch adds `g + c` once to each nominal torque
controller, and removes gravity compensation from the projected task/nullspace
terms. This is an explicit software compensation correction. It does **not**
describe a measured Franka torque-to-acceleration mismatch or alter the mass
matrix. [Commit message and diff](https://github.com/StanfordASL/oscbf/commit/88ae88f708f9308ed6fa8e6e16690012f127c7c8)

### July 8, 2025: model-error sensitivity in the dynamic-motion demo

Commit `b16f4429dc9d06ce711cb94cb76b2cb5dc5d912b` explicitly attributes degraded
dynamic-demo performance to including centrifugal/Coriolis terms in the presence
of model error. It adds `compensate_centrifugal_coriolis` to the torque CBF
configuration and sets that flag to `False` in `dynamic_motion.py`. With the
flag false, the demo zeros `c` in nominal torque generation and omits `c` from
the CBF drift, while retaining gravity. The general configuration default
remains `True`. [Commit message and diff](https://github.com/StanfordASL/oscbf/commit/b16f4429dc9d06ce711cb94cb76b2cb5dc5d912b)

**Interpretation:** the authors did acknowledge dynamics-model error in this
simulation demo and addressed its effect by making C/C use consistent and
optional. This does not identify their hardware's one-step acceleration error,
and does not support copying the switch as a solution to the local low-speed,
large-scale discrepancy. The patch changes the bias term, not inertia. [Same
patch](https://github.com/StanfordASL/oscbf/commit/b16f4429dc9d06ce711cb94cb76b2cb5dc5d912b)

For local recorded cycle 22,810, the computed joint-7 contribution
`[M^-1*c]_7` is only **0.04119 rad/s²**, compared with the roughly 455 rad/s²
prediction. Omitting C/C would therefore not remove this particular discrepancy.
This is a calculation from the saved local mass/Coriolis data, not a claim from
the OSCBF authors.

### Infeasibility is discussed separately

The paper acknowledges conflicting CBF/input constraints and proposes penalized
slack, expressly giving up a safety guarantee under relaxation. It also explains
that torque limits can make reduced-order velocity commands infeasible. Neither
discussion diagnoses a plant-model scale error. [Paper v3, §II and §V.D,
pp.2–3,7](https://arxiv.org/pdf/2503.06736v3#page=2)

## Public issue, pull-request and history audit

The following API results were read with `state=all` and `per_page=100`, covering
both open and closed entries. Each returned fewer than 100 items. Counts are a
snapshot at the review date, not immutable historical facts.

| Source | Inspected public record | Relevance |
|---|---|---|
| [OSCBF issues](https://api.github.com/repos/StanfordASL/oscbf/issues?state=all&per_page=100) | Two closed issues and all three issue comments | #1 is Ubuntu 20.04/Python 3.8 compatibility; #2 is JAX/XLA/C++ deployment and the hardware-workspace link. No acceleration-model report. |
| [OSCBF pull requests](https://api.github.com/repos/StanfordASL/oscbf/pulls?state=all&per_page=100) | Empty result | No public PR discussion to supply such a diagnosis. |
| [Hardware-workspace issues](https://api.github.com/repos/StanfordASL/oscbf_hardware_ws/issues?state=all&per_page=100) and [pull requests](https://api.github.com/repos/StanfordASL/oscbf_hardware_ws/pulls?state=all&per_page=100) | Both empty | No public hardware issue/PR diagnosis found. |
| [CBFpy issues](https://api.github.com/repos/StanfordASL/cbfpy/issues?state=all&per_page=100) and [pull requests](https://api.github.com/repos/StanfordASL/cbfpy/pulls?state=all&per_page=100) | One issue, three PRs; eight conversation comments, and inline-comment endpoints for code/dependency PRs #2/#4 | Desired-state input in CLF evaluation, a logo, and NumPy/PyBullet compatibility. No Franka acceleration mismatch. |
| [OSCBF commit history](https://api.github.com/repos/StanfordASL/oscbf/commits?per_page=100) | 38 commit messages; both relevant compensation diffs read | The March/July changes above are the explicit dynamics-related correction reports. All returned commits have zero commit comments. |
| [Hardware-workspace commit history](https://api.github.com/repos/StanfordASL/oscbf_hardware_ws/commits?per_page=100) | 16 commit messages | Initial hardware-test code, dependency/refactoring work, and adding velocity CBFs/adjusting singularity tolerance; no message reports a large acceleration-scale correction. All returned commits have zero commit comments. |

Author comments underlying the non-dynamics issues are available at
[OSCBF #1](https://github.com/StanfordASL/oscbf/issues/1#issuecomment-3629736135),
[CBFpy #1](https://github.com/StanfordASL/cbfpy/issues/1#issuecomment-2669628391),
and [CBFpy #4](https://github.com/StanfordASL/cbfpy/pull/4#issuecomment-3461270352).
No GitHub Releases were listed for either
[OSCBF](https://api.github.com/repos/StanfordASL/oscbf/releases?per_page=100) or
[the hardware workspace](https://api.github.com/repos/StanfordASL/oscbf_hardware_ws/releases?per_page=100).

## Source-code comparison with our filter

The following findings concern the pinned public release, including its linked
hardware workspace. They do not establish the exact dependency versions used
in every historical paper experiment. No third-party controller was executed,
and no local control/configuration implementation was changed.

### 1. Same basic forward dynamics, different torque coordinate

`OSCBFTorqueConfig.f()` builds the drift with gravity and optional C/C, and
`g()` supplies inverse mass. Together they implement
`qdd=M^-1*(Gamma-c-g)` for a total-torque optimization variable Gamma.
[Pinned config, lines 81–105](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/oscbf_configs.py#L81-L105).

The hardware `compute_control()` filters that total torque, then returns
`Gamma-g` to Franka. Its comment explicitly explains avoiding double gravity
compensation while maintaining the intended torque limits.
[Hardware conversion, lines 165–172](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control/scripts/franka_control_node.py#L165-L172).

With `u=Gamma-g`, their acceleration equation becomes `M^-1*(u-c)`, the same
gravity-compensated coordinate used locally. Their explicit gravity term is
therefore not evidence that our command-coordinate equation should subtract
gravity again. Bounds differ: their full-torque bound transforms to
`-tau_max-g <= u <= tau_max-g`, not automatically a symmetric bound on u.

### 2. Hardware uses the bundled URDF model, not libfranka mass

The Python hardware node calls `load_panda()`, which builds the model from
`oscbf/assets/franka_panda/panda.urdf`. The C++ wrapper forwards joint states and
torque commands; its libfranka model pointer is commented out. Thus the inspected
hardware QP does not consume `model.mass(robot_state)`.
[Hardware model creation](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control/scripts/franka_control_node.py#L217-L222),
[load_panda](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/manipulator.py#L909-L923),
[C++ wrapper](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control/src/franka_impedance_controller.cpp#L198-L233).

Its mass calculation sums ordinary rigid-body translational and rotational
inertias from the URDF. No explicit apparent-motor-inertia matrix or actuator
identification term was found in this path.
[Mass implementation](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/manipulator.py#L767-L794).

The supplied URDF assigns **0.1 kg m² to each diagonal inertia entry** of the arm
links and Hand/finger/fixed distal bodies. The parser combines fixed-body
inertias, retaining inertia even on its zero-mass fixed links.
[URDF distal bodies](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/assets/franka_panda/panda.urdf#L191-L340),
[fixed-body inertia merging](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/utils/urdf_parser.py#L560-L609).

An independent analytic rigid-body calculation for their fixed-gripper URDF
gives joint-7 diagonal inertia **M77=0.60002 kg m²**. It comprises six distal
0.1 contributions plus the two fingers' parallel-axis contributions
`2*0.1*(0.01)^2`. All intervening fixed transforms preserve the joint-7 axial
direction. Our recorded libfranka value at cycle 22,810 is **0.00296161 kg m²**,
approximately 203 times smaller. This diagonal ratio is not an acceleration
ratio for a coupled matrix; it establishes a major difference in model inputs.
It does not demonstrate that either value is the calibrated effective inertia of
the real closed-loop robot, or that the authors intentionally used these values
as motor-inertia compensation.

The URDF provenance notes identify a PyBullet robot asset with parser/geometry
edits, not a hardware actuator-parameter calibration.
[Asset README](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/assets/franka_panda/README.md).

The dynamics tests compare the same URDF against Pinocchio and PyBullet, with
PyBullet explicitly instructed to use the file's inertias. These validate
implementation agreement for the supplied model; they do not compare it to
libfranka's matrix or identify hardware acceleration response.
[Dynamics tests](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/test/test_manipulator_dynamics.py#L17-L117).

### 3. The nominal controller is inertia-shaped operational-space control

Their nominal pose controller first constructs a task acceleration, multiplies
it by operational-space inertia `Lambda=(J*M^-1*J.T)^-1`, then maps its wrench
through `J.T`. Its nullspace contribution uses `M*joint_accel`. It adds gravity
and C/C once. Our primary Cartesian nominal law maps a virtual spring/damper
wrench through `J.T` directly, with a separate projected posture torque.
[OSCBF controller, lines 151–176](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/controllers.py#L151-L176).

This is a substantive controller-design difference relevant to acceleration
shaping. It does not, by itself, establish physical prediction accuracy or
resolve an infeasible fixed-state input set. The different inertial parameters
affect both their nominal controller and their barrier model.

### 4. Gains, bounds and post-processing differ

Their hardware `DemoConfig` uses alpha and alpha2 of **10**, the URDF's joint
position/velocity limits, and the full URDF torque-magnitude bounds. The inspected
QP setup does not include our previous-torque-centred **±0.35 Nm/1 ms** interval.
Our active setup uses joint gains 5, explicit position/velocity margins and
35%-scaled torque/rate ceilings. These are different feasible sets.
[Hardware barriers/gains](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control/scripts/franka_control_node.py#L59-L118),
[QP input bounds](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/oscbf_configs.py#L73-L79).

The C++ wrapper invokes `robot.control(callback)` with default signal-processing
arguments. In the linked patched libfranka 0.8.0, those defaults enable native
rate limiting and a **100 Hz** command low-pass filter. Locally, the callback
explicitly selects `kMaxCutoffFrequency`, disabling that command low-pass filter.
No equivalent QP slew constraint or post-processing revalidation was found in
the inspected OSCBF hardware control path. This is a comparison, not a
recommendation to remove local command verification.
[Wrapper call](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control/src/franka_impedance_controller.cpp#L203-L224),
[linked libfranka defaults](https://github.com/danielpmorton/libfranka_08_patch/blob/41348a83dadd789e437b0133746bede118b0a24d/include/franka/robot.h#L149-L153),
[cutoff value](https://github.com/danielpmorton/libfranka_08_patch/blob/41348a83dadd789e437b0133746bede118b0a24d/include/franka/lowpass_filter.h#L20-L25).

The linked libfranka patch is for build compatibility: the inspected changes add
standard-library includes and correct the version to 0.8.0. They do not change
inertia or torque dynamics.
[Import patch](https://github.com/danielpmorton/libfranka_08_patch/commit/bdaea328a42c799ed45393d8d6913472ab030067),
[version patch](https://github.com/danielpmorton/libfranka_08_patch/commit/faa0219f286d857f01fa2097389ae6a23a4d9350).

### 5. The public dependency permits elastic relaxation

The main package pins `cbfpy==0.0.1`. The published 0.0.1 source archive was
checked against PyPI's SHA-256
`05c4795467d4de60396e61c1661068c7ded1ec54e08b708becd3596f60889a65`.
Its constructor defaults are `relax_cbf=True`, a scalar relaxation penalty of
1000, and solver tolerance 1e-3. The hardware configuration inherits these
defaults without overriding them.
[Dependency pin](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/pyproject.toml#L18-L20),
[published-package metadata](https://pypi.org/pypi/cbfpy/0.0.1/json),
[matching initial-release source defaults](https://github.com/StanfordASL/cbfpy/blob/e89c94f388bd17201c84c695a29012e5a01b4b74/cbfpy/config/cbf_config.py#L79-L88).

That CBFpy path selects an elastic solver and returns the torque vector without
rejecting positive slack or an unsuccessful convergence flag. The actuator-box
rows are appended to the same inequality matrix passed to that solver.
[Solver handling](https://github.com/StanfordASL/cbfpy/blob/e89c94f388bd17201c84c695a29012e5a01b4b74/cbfpy/cbfs/cbf.py#L173-L197),
[inequality assembly](https://github.com/StanfordASL/cbfpy/blob/e89c94f388bd17201c84c695a29012e5a01b4b74/cbfpy/cbfs/cbf.py#L310-L342).

The contemporary QPAX implementation uses one nonnegative relaxation variable
per row of that matrix. Thus this historical combination can soften both
barrier and actuator inequalities. QPAX itself is unpinned in the OSCBF package,
so this is corroborating historical solver semantics, not proof of the exact
installed dependency during a paper experiment.
[Historical elastic equations](https://github.com/qpax-solver/qpax/blob/53df513a79909854a5fb4e4d92948e4984e8488b/qpax/elastic_qp.py#L104-L114).

Our implementation instead enforces the actuator interval as hard bounds,
rejects positive barrier slack, and independently checks outgoing torque. A
returned OSCBF solution therefore is not equivalent to a locally accepted
zero-slack command. No inspected evidence establishes substantial positive slack
in their hardware demonstrations; the distinction is the acceptance policy.

## Reproducibility and remaining evidence gap

- OSCBF default branch `main` at review:
  [`6082329f8e1d61ee8e875a63a5cd80a14d513e12`](https://github.com/StanfordASL/oscbf/tree/6082329f8e1d61ee8e875a63a5cd80a14d513e12).
- Hardware workspace default branch `master` at review:
  [`26f251f6956450475031ab906ebb08f3d675e73a`](https://github.com/StanfordASL/oscbf_hardware_ws/tree/26f251f6956450475031ab906ebb08f3d675e73a).
- Paper pinned to [arXiv v3](https://arxiv.org/abs/2503.06736v3), October 18, 2025.
- Project page and issue/API lists are live sources; their review date is above.

The inspected statements do not provide raw synchronized commanded/applied
torques and joint-acceleration traces, a quantitative forward-dynamics residual
study, or a documented motor-inertia identification. Consequently, they cannot
confirm or reject the local missing-apparent-inertia hypothesis. Any comparison
with the public hardware source must still distinguish nominal closed-loop
tracking, barrier evaluation, torque-command compensation and actual measured
plant acceleration. No local controller or configuration was changed for this
review.
