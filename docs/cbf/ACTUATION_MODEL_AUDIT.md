# Torque-to-acceleration discrepancy audit — 2026-11-05

The current predictor disagrees strongly with recorded motion and is not a
validated physical safety model for this installation. The operator confirms
no physical contact or noticeable cable pull during the free-space trial.
The leading new hypothesis is omitted actuation dynamics, particularly apparent
motor inertia and the internal torque loop. It is not a confirmed hardware fix.
No production dynamics, barrier gains or actuator limits were changed.

## Reproduction from the actual emitted constraints

Run from the repository root:

```bash
python3 log/commissioning/2026-11-05-cbf-forensics/review/audit_torque_balance.py --check
```

Expected exit status is 1: the original prediction mismatch is reproduced.
The audit reconstructs acceleration from the upper velocity rows captured from
the actual runtime builder, independently compares it with `M^-1*(tau-c)`, and
checks recorded velocity and position changes. Agreement between the two
prediction calculations is <1e-9. Its 5 rad/s² diagnostic allowance is not a
deployment safety tolerance. The physical prediction check remains red.

For cycle 22,810, using the surrounding 20 ms and freezing the snapshot M/c:

| Joint-7 calculation | Acceleration (rad/s²) |
| --- | ---: |
| Original runtime row with mean desired-torque readback | 455.879 |
| Same model with mean actually issued software command | 457.439 |
| Same model with measured torque minus modeled gravity | 277.545 |
| Subtract full modeled gravity again from desired torque | 467.047 |
| Recorded velocity change over the window | 0.407 |
| Quadratic fit to recorded positions over the window | -0.080 |

Position fitting and differentiated velocity are different noisy/filtered
estimates; neither is a calibrated acceleration sensor. Their small magnitudes
nevertheless do not support the predicted hundreds of rad/s². Merely replacing
desired torque by measured torque does not repair the tested rigid model.

Over the 32 captured states, the median absolute joint-7 acceleration error is
167.961 rad/s² with desired torque and 161.966 with measured-minus-gravity torque.
Shifting the desired-torque window into the past by 1, 2, 5, 10, 20 or 50 ms
does not remove the discrepancy. This tests a simple delay hypothesis; it does
not exclude general actuator/observer dynamics. The position-fit joint-7
accelerations remain below 3.455 rad/s² in these windows.

At the example state, the rigid model and observed velocity slope require only
0.00155 Nm of gravity-free net torque at joint 7. Mean desired torque is 0.63406
Nm; mean measured torque minus modeled gravity is 0.42935 Nm. These leave signed
balance residuals of 0.63251 and 0.42780 Nm respectively. They are unexplained
residuals, not identified friction or external torque. Small modeled inertia
amplifies these sub-Nm residuals into very large acceleration errors.

## Specific actuation-model hypothesis

The implementation currently maps the API torque variable u directly to link
acceleration through `M^-1*(u-c)`. Gravity cancellation remains appropriate for
the documented gravity-compensated command convention; it does not eliminate
the dynamics of the motor, transmission and low-level torque controller.

A manufacturer-coauthored study derives coupled motor/link dynamics and a
reduced model using an effective matrix `M+B_theta`, where the added term depends
on motor inertia and torque-loop gains. Its assumptions include a particular
inner torque law and a rigid-joint approximation; the experimental focus is
post-impact response. This motivates a hypothesis for our predictor rather
than certifying that model for our free-space hardware or firmware.
[Arias et al., 2024, Sections II-A/II-C](https://pure.tue.nl/ws/files/320059449/Refined_Post-Impact_Velocity_Prediction_for_Torque-Controlled_Flexible-Joint_Robots.pdf).

For illustration, under that approximation and ideal compensation, the predictor
would have the form

`a = (M+B_eff)^-1 * (u-c + compensation/friction/external residuals)`.

The finite-stiffness version also depends on motor acceleration. Our current
records do not contain the motor position/velocity trajectory or the identified
inner-loop parameters needed to validate it. The exact dynamic scope of the
downloaded libfranka model also needs confirmation before adding an inertia term;
we must avoid counting any modeled contribution twice.

## Offline sensitivity, not hardware calibration

An independent sensitivity uses the approximate diagonal values reported in
van Steen's thesis, Section 4.2/Table 4.1:

`B_theta = diag(0.2, 0.2, 0.15, 0.15, 0.067, 0.067, 0.067) kg m²`.

These are literature parameters, not measured settings for this robot.
[Primary thesis](https://www.vandewouw.dc.tue.nl/phd_thesisvansteen2025.pdf).

For each captured state, all geometry, gains, nominal torques, command intervals
and slack policies are retained. Only the hypothesized inertia changes. With
`G=R*M^-1` and `b=b_kin+G*c`, the offline reconstruction changes these coefficients
consistently. The actual C++ builder agrees with the reconstructed coefficients
to <1e-9. The selected qpOASES/elastic solver and outgoing verification then
independently check all 32 states.

| Quantity | Original | Reduced-motor-inertia sensitivity |
| --- | ---: | ---: |
| Example mean joint-7 prediction | 455.879 | 9.567 rad/s² |
| Feasible captured problems | 6/32 | 32/32 |
| Feasible bounded nominal commands | 0/32 | 31/32 |
| Median absolute joint-7 prediction error | 167.961 | 7.106 rad/s² |

All 32 augmented native solutions pass current outgoing verification. This is
not an acceptance-rate estimate: the captures were selected from failed cycles.
The example prediction still disagrees with the observed 0.407 rad/s², so this
does not close the physical prediction failure. Inflating inertia arbitrarily
would also suppress predictions; an improvement in these metrics alone cannot
identify the correct physical parameters. The published mechanism supplies a
reason to investigate the hypothesis, not permission to deploy its constants.

The unreduced-inertia alternative from the same table gives an example prediction
of 3.186 rad/s² and lower errors here. That does not justify using it: selecting
whichever inertia makes the current traces look best would be unvalidated tuning.

## Conclusions and remaining evidence

1. The predictor as applied to this robot is not validated. Accurate row algebra
   does not establish an accurate command-to-link-motion model.
2. Omitted motor/inner-loop dynamics are a concrete leading candidate, supported
   by primary literature and a substantial offline sensitivity, rather than a
   conclusion that libfranka's rigid-body matrix is itself grossly inaccurate.
3. Residual friction, compensation/load error and sensor/observer dynamics remain
   relevant. The recorded command-versus-sensor difference alone does not account
   for the full discrepancy; simple delay or extra gravity subtraction does not
   resolve it either.
4. Additional discriminating quantities are high-rate motor theta/dtheta,
   measured torque derivative, external-torque estimate and configured end-effector
   and load properties, alongside the already captured q/dq/M/c/gravity and commands.
   Those missing quantities cannot be recovered retrospectively from this bag.
5. Any correction must update the effective torque-to-acceleration map and drift
   consistently for all enabled barriers, retain actuator/deadline checks, and
   validate physical response independently of numerical acceptance. Inverse
   dynamics using the same unvalidated model does not provide that validation.

## Evidence

`log/commissioning/2026-11-05-cbf-forensics/review/` contains:

- `audit_torque_balance.py`, `torque-balance-audit.json`, and the minimized
  `torque-balance-cycle-22810.json` red reproduction.
- `evaluate_model_hypotheses.py`, `model-hypothesis-summary.json`, and per-state
  details for delay, signal and apparent-inertia sensitivities.
- `native_actuation_probe.cpp`, `native-actuation-probe.jsonl`: 64 actual-library
  cases, original and augmented, with independently checked row coefficients.
- `sources/van_steen_2025.pdf` and text extraction for private research inspection.

All investigation was offline. Synchronous selection and production model
settings are unchanged by this audit.
