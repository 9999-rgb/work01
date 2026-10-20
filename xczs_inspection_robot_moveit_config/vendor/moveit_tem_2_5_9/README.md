# MoveIt 2.5.9 trajectory-execution shutdown fix

Vendored BSD-3-Clause translation unit from the upstream 2.5.9 tag. The only
code change is the destructor release order in `shutdown.patch`, adapted from
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
