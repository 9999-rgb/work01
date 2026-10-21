# MoveIt 2.5.9 shutdown fixes

Vendored BSD-3-Clause translation unit from the upstream 2.5.9 tag. Its trajectory-execution
change is the destructor release order in `shutdown.patch`, adapted from
[upstream PR #3828](https://github.com/moveit/moveit2/pull/3828) (still open when
reviewed on 2026-10-19). Source and header hashes are in `provenance.json`.

The original plugin loader can unload its library before the internal node
releases weak action-client references. Destroy the clients, executor, and node
while the loader is still alive. Cancellation and thread joining are retained.
No trajectory generation, timing, execution limits, or class layout changes.

CMake checks both the installed version and exact public-header hash before
building this same-SONAME replacement. It installs into this package's private
`lib/xczs_moveit_tem_fix` directory. Only the project move_group process gets
that directory prepended to LD_LIBRARY_PATH; /opt/ros is not overwritten.

`test_shutdown.cpp` creates a real FollowJointTrajectory action server and
loads a controller-manager plugin with one action client. It verifies teardown
both before and after ROS context shutdown. The unpatched installed library
segfaulted (-11); the patched library exited normally in the isolated probe.

## Capability plugin lifetime

`move_group.cpp` retains the upstream 2.5.9 program and its BSD license. The
small `move_group_shutdown.patch` keeps a second shared owner of the capability
loader, declared before the main node. The loader is consequently released
after the executor, MoveIt objects, and the node's callback groups. This avoids
unmapping default-capability weak-reference control blocks before their final
destruction. No planner, capability, action, or execution behavior is changed.
The executable is installed privately under this package; `/opt/ros` stays intact.

The failure was reproduced with the installed executable and the TEM fix already
loaded. GDB identified the dangling vtable address inside the now-unmapped
`libmoveit_move_group_default_capabilities.so.2.5.9`. The private executable
exited cleanly in 10 consecutive isolated probes. CTest also loads the actual
A/B robot configurations and real capabilities, sends SIGINT after readiness,
and requires exit code zero. Provenance records both upstream and patched hashes.

A rapid hot-switch also exposed SIGINT during planner initialization: ROS had
already invalidated its context while a publisher was being constructed. The
program handles `RCLError` only after ROS shutdown has occurred; errors with a
live context still propagate. The startup-cancellation CTest interrupts actual
pipeline loading and requires a clean exit.
