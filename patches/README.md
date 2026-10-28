# Pinned dependency patches

`libfranka-0.9.2-hand-stop.patch` keeps the pinned 0.9.2 protocol and public API.
It changes the Hand connection's blocking response wait to poll outside the
shared TCP send/read mutex, so an unfinished `grasp` or `move` cannot monopolize
that mutex and delay `stop`. Responses are copied/removed under the mutex, and
closed or undersized replies raise an exception. The arm connection retains its
existing response-polling cadence; its torque loop and limits are unchanged.

The SDK checkout is a nested dependency repository. Its local edits alone are
not included in the parent repository's commits. This tracked patch and its
application script make the correction reproducible after a fresh import.
`scripts/fetch_teleop_dependencies.sh` applies it automatically. For an existing
checkout, run this from the workspace root before rebuilding:

```bash
bash scripts/apply_teleop_dependency_patches.sh
```

The script accepts an already applied patch and refuses conflicting SDK changes.
Do not update the pinned SDK or discard its local edits as a workaround.

The `teleop_gripper` test `test_hand_transport` uses the actual linked SDK and a
loopback fake Hand at `127.0.0.1:1338`. It tests Stop while grasp/open remains
unfinished, fragmented/reordered replies, malformed replies and disconnection.
It never addresses a robot. Passing it establishes software transport behavior,
not firmware response time, mechanical stopping distance or force accuracy.

The SDK's existing Apache-2.0 license continues to apply.
