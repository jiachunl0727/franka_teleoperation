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

`libfranka-0.9.2-startup-observer.patch` adds the opt-in
`Robot::controlWithStartupObserver` entry point for paired CBF startup. The
ordinary `control()` path is unchanged. On the existing control-owner thread and
connection, the new path polls the initial Move acknowledgement while consuming
new UDP states, and exposes those states and the subsequent mode transition to
a bounded observation callback. The callback cannot supply torque or motion
commands. After startup, it is cleared and ordinary torque callbacks take over.
The pending startup is bounded to one second plus existing receive/cancellation
timeouts. A startup error attempts Stop/cancellation and propagates; no automatic
recovery or unchecked holding command is provided.

The runtime uses this callback only for inter-arm-enabled hardware, validates
Idle/Move without current errors and stationary feedback, and rebuilds/publishes
WORLD geometry from each newly received state with its actual acquisition stamp.
It latches pair failure before cancellation if observation fails. Original peer
age/skew and CBF command checks remain active; this is not a stale-data grace
period. A transport/scheduling gap exceeding the freshness budget still stops
the pair.

`test_inter_arm_startup_transport` uses the SDK's loopback fake robot at
`127.0.0.1:1337`. It delays the Move acknowledgement while sending changing UDP
states, checks native constraint preparation with new versus stale observations,
and checks exclusive ownership, ordinary completion, observer-fault
cancellation and rejected acknowledgements before any torque callback. These tests establish software behavior,
not hardware startup timing or physical collision protection.

`libfranka-0.9.2-receive-freshness.patch` drains newly queued states after a
blocking receive, including when the receiving thread was descheduled during
that call. Selection retains the greatest actual message ID and its robot time;
it never refreshes an old state's timestamp. A drain is bounded to 32 datagrams;
an excessive backlog raises a network exception instead of starving callback
entry. This bounds queue-draining work, not network wait or scheduler latency.

The patch also adds local `Robot::controlTiming()` diagnostics: steady-clock
update/receive boundaries, update CPU time, datagram count and whether the
receive waited. The getter is for the existing owner thread inside its control
callback; it never locks or reads the socket. RobotState layout and the FCI wire
protocol are unchanged. Rebuild the SDK and teleoperation controller. Normal and
terminal CBF diagnostics record these fields under `fci_timing`; mock data has
null timing rather than fabricated receive measurements. Receive wall time
includes normal inter-packet waiting and possible descheduling, not just network
latency. A loopback regression injects a receiver delay, queues reordered newer
states and requires the newest state; another checks bounded backlog failure.
