# Humble TF2 deadlock repair

Official source: https://github.com/ros2/geometry2/tree/0.25.24
Archive: https://codeload.github.com/ros2/geometry2/tar.gz/refs/tags/0.25.24
Fix: https://github.com/ros2/geometry2/pull/990

The upstream archive includes its original licenses. SHA256SUMS pins the archive.
Only tf2 and tf2_ros are built. The image installs these packages into
/opt/ros/humble using per-package CMake installation, preserving the existing
ROS setup scripts. This deliberately replaces apt-provided package files;
dpkg still reports the original deb versions. Do not apt-upgrade TF2 in the
running container: rebuild the image to retain this reproducible fix.

The build runs upstream tf2_ros/test/test_buffer.cpp, including
wait_for_transform_does_not_deadlock_with_set_transform, and checks that the
test actually links the newly installed libraries. Successful builds record
/opt/ros/humble/share/xczs-tf2-fix/build.txt.
