# Native release-braking regression fixture

Source: right terminal at cycle 8784 in
`quest_20261116T163049Z_53f83697`, recorded 2026-11-16. Bag SHA256:
`3c87bb9c3a4b78c73b7dbb655af1d3bb4f55988f5756b4c051fc41c88863d511`.

The native solution is feasible under the existing verification tolerance,
but joint 2 exceeds its actuator upper bound by approximately 8.0e-14 Nm.
The exact clamped outgoing command passes verification. There are no retained
inter-arm rows; the correction is local point-acceleration limiting.

`state.txt` contains q, dq, nominal, lower/upper torque bounds, Coriolis,
row-major mass matrix, peer count/speed/acceleration/age in ns, then each peer
sphere's position, velocity and radius. Frozen configuration/geometry files
make the case independent of operator-edited deployment parameters. FK/J use
the repository's UMI URDF; runtime M/c are restored from the recorded state.
Offline timing limits are enlarged only in the test to isolate policy from
machine execution speed.
