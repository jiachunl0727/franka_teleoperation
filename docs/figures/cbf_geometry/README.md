# Exact CBF capsule and bounding-box placement

**Historical figure:** this predates the current UMI attachment and CBF layout.
Use the [interactive collision viewer](../collision_model/viewer.html)
for the current dual-arm geometry. The placement data below describes the older model.

Generated from the current C++ initializer and the pinned Panda URDF/Xacro.
All 14 capsule definitions were checked against the URDF cylinder/sphere
collision primitives at `safety_distance=0` before rendering.

## Posture and coordinate convention

- Common home joints from `teleop_bringup/config/single_panda/single_panda.yaml`:
  `[0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854]` rad.
- The illustration world is the robot base frame (identity base transform).
  Joint poses come from the pinned URDF; this is not a live robot snapshot.
- All 14 capsules are drawn, regardless of configured feature switches.
  Optional-CBF-off operation only computes the mandatory table capsules
  (C02–C13), excluding mounting geometry C00/C01.
- Gray geometry is the original URDF visual mesh for the arm and hand
  housing. Finger meshes are omitted; the CBF has no detailed, width-varying
  finger or cable collision geometry.

## What the boxes mean

A capsule is a line segment swept by a sphere of radius `r`.
`L` below is the centerline segment length; total end-to-end extent is `L+2r`.
For local center `c`, unit direction `u`, and frame-to-world transform `T`:

```text
A = T * (c - (L/2) u)
B = T * (c + (L/2) u)
box_min = componentwise_min(A, B)
box_max = componentwise_max(A, B)
```

The stored AABB encloses the centerline, not the complete capsule. Thus it
may be flat, a line, or (for C10/C11) a point. It remains parallel to the
world axes while its dimensions change with the measured joint posture.
The radius-expanded dashed envelope in the detail figure is explanatory;
the code stores the unexpanded box and adds radii, clearance and reserves
to the pair-distance rejection threshold separately.

The source URDF wrapper requests `safety_distance=0.03` for its collision
display. The CBF intentionally transcribes the zero-distance primitive
sizes and adds its own margins once; the illustrations show that CBF model.

## Exact local definitions

Coordinates, segment lengths and radii in this table are **millimeters**.
Centers and directions are expressed in the stated attachment frame.
C12/C13 use the flange frame after the hand mounting yaw of −45°.

| ID | Collision link | Attachment frame | Center (x, y, z), mm | Unit axis | L, mm | r, mm |
|---|---|---|---|---|---:|---:|
| C00 | link0 | base | (-75, 0, 60) | (1, 0, 0) | 30 | 60 |
| C01 | link1 | link1 | (0, 0, -191.5) | (0, 0, 1) | 283 | 60 |
| C02 | link2 | link2 | (0, 0, 0) | (0, 0, 1) | 120 | 60 |
| C03 | link3 | link3 | (0, 0, -145) | (0, 0, 1) | 150 | 60 |
| C04 | link4 | link4 | (0, 0, 0) | (0, 0, 1) | 120 | 60 |
| C05 | link5 | link5 | (0, 0, -260) | (0, 0, 1) | 100 | 60 |
| C06 | link5 | link5 | (0, 80, -130) | (0, 0, 1) | 140 | 25 |
| C07 | link6 | link6 | (0, 0, -30) | (0, 0, 1) | 80 | 50 |
| C08 | link7 | link7 | (0, 0, 10) | (0, 0, 1) | 140 | 40 |
| C09 | link8 | link8 | (42.4, 42.4, -25) | (-0, -1, -0) | 10 | 30 |
| C10 | link8 | link8 | (42.4, 42.4, -20) | (sphere) | 0 | 30 |
| C11 | link8 | link8 | (42.4, 42.4, -30) | (sphere) | 0 | 30 |
| C12 | hand | link8 | (0, 0, 40) | (0.707107, 0.707107, 0) | 100 | 40 |
| C13 | hand | link8 | (0, 0, 100) | (0.707107, 0.707107, 0) | 100 | 20 |

## Exact world coordinates and regeneration

[CSV](placement.csv) and [JSON](placement.json) include local definitions,
world endpoints and stored AABB minimum/maximum corners in **meters**,
with unrounded floating-point values.

From the repository root, with NumPy, Matplotlib and PyYAML available:

```bash
python3 scripts/illustrate_cbf_geometry.py
```

This reads source files and writes documentation artifacts only.
It makes no ROS, network, gripper or robot connection.
