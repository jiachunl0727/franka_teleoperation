# Full-arm enforcement and Franka hand housing coverage

The operator reported avoidance at the fingertips but insufficient avoidance at
the Franka hand housing. The independent predictor already iterates every own
body sphere against every peer sphere; there is no fingertip-only mask. Source
groups cover the base, links 1–7, flange, hand and UMI extensions. Held owners
retain their existing local holding/safety output. An active owner avoids the
complete peer geometry, including its base and held links.

The concrete defect was in the housing approximation. Its original spheres
came from the pinned description's simplified collision primitives. Against the
same description's `meshes/visual/hand.dae`, 2559 of 3659 vertices lay outside the
hand sphere union; the largest uncovered distance was 35.452 mm. The hand-to-flange
transform agrees with the model's fixed Rz(-pi/4), so no transform change is made.
This geometric omission is a plausible cause of the reported housing behavior;
no recording of that specific encounter was provided to establish its cause.

The generator now also encloses the full housing mesh bounding box in eight
cells (four along its long axis and two along its axial direction). Each cell's circumsphere encloses the entire cell,
including triangle interiors. Offsets are rotated into the existing flange
frame. The eight added group-9 spheres increase stock geometry from 43 to 51 and
the UMI-equipped dual geometry from 55 to 63 spheres per owner, within the
64-sphere capacity. The stock/UMI single-arm geometry and both dual profiles are
updated consistently. The current sphere approximation, transforms, clearance,
10 ms prediction and freshness limits, holding policy and timing guards remain.
No additional unexpected-acceleration allowance is introduced.

The new C++ tests use the real Panda model and geometry. They account for every
own/peer pair in both directions (63*63 = 3969 per direction), compare grouped
pruning against exhaustive evaluation, and place an obstacle next to every
movable sphere. A nominal torque that violates that sphere's inter-arm row must
be changed and the issued torque must verify. Stationary base/axial first-link
spheres cannot move themselves; the approaching active arm avoids those shapes.
The housing coverage regression checks all mesh vertices and all eight corners
of every enclosing cell in all four shipped geometry files. It failed on the
old cover and passes with the added spheres.

This change strengthens inter-arm and self-collision body geometry. Existing
table capsules are a separate model and are unchanged. Mesh coverage here refers
to the supplied Franka description, not a measurement of physical hardware or
an additional mounted tool. The cell cover can be conservative around corners.
No hardware trial was performed.
