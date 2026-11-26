# Capsule body cover and moving UMI fingers

Open the [collision viewer](../figures/collision_model/viewer.html) and choose a part under **Inspect body volume**.
The opening slider moves a sphere with each stock finger and a fitted capsule
with each UMI fingertip. The UMI capsule preview radius is 12.9 mm after
reducing the original fitted radius by 2 mm. Its lower end was shortened by 12 mm in the
home posture to reduce overhang during tip-to-tip work.
Runtime adds an allowance for motion between width samples.

The body model starts with the stock arm and wrist table capsules and merges
overlapping volumes on each rigid link. Link 5 uses two offset capsules for
its separate sections. One transverse capsule covers the Hand housing. Its
current preview radius is 5 mm smaller than the original enclosing fit.
Table checks use the same cover as body collision, excluding the base and Link 1. This includes the moving stock finger
spheres and UMI capsules. The viewer's **Table collision cover** toggle shows
this subset.

The base cover uses one 65 mm sphere at `[-90, 0, 60]` mm in the base frame.
The Link 1 sphere centered at world Z = 0 mm in the home posture is omitted
from this cover. One axial Link 1 capsule contains the remaining spheres.

The [single](../../src/cbf_safety_filter/config/single_panda/cbf.yaml) and
[dual](../../src/cbf_safety_filter/config/dual_panda/cbf.yaml) launch profiles
load the reviewed `body_volumes` entries directly. Both use snapshot recovery.
The geometry is shared across the projection and native QP torque methods.

The Hand worker publishes measured total width in metres on its namespaced
`gripper_width` topic, using a dedicated UDP reader. The arm owner reads the
latest sample through a fixed-size handoff within its callback. Each UMI capsule
moves along its URDF finger axis by half the measured width change. The CPU
monotonic timestamp must originate on the same host as the arm owner. The
radius includes width measurement and travel allowances, using a per-finger
speed bound of 0.2 m/s, sample age and peer snapshot lifetime. The Hand worker
publishes this feed for configured total opening/closing speeds within that
bound. Missing, invalid, future or older-than-30 ms samples use each finger's
whole 0–40 mm travel interval.

Body checks use the closest pair of centerline points to choose a normal for
capsule pairs. The CBF then checks the projected gaps at the segment endpoints
along that normal. This bounds the projected gap for the entire segments and
also covers endpoint transitions and parallel centerlines. This separation
constraint can be more conservative than a differentiated closest-point
distance. Sphere-only pairs use the previous distance equation. The existing
recovery allowances and torque verification still apply. The table checks the
two endpoints of each selected capsule and the center of each selected sphere.

Regenerate the Hand and fingers after editing their meshes. Edit the reviewed
arm volumes in `geometry_panda_umi.yaml`. The generator copies that arm cover
to both dual geometries. Run these from the repository root:

```bash
python3 scripts/generate_capsule_body_cover.py
python3 scripts/visualize_umi_model.py
```

