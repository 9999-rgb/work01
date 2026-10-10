ARG ROS_IMAGE=m.daocloud.io/docker.io/library/ros:humble-ros-base-jammy
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple \
    PIP_DEFAULT_TIMEOUT=60 \
    ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
COPY scripts/deploy/configure_mirrors.py /tmp/configure_mirrors.py
RUN python3 /tmp/configure_mirrors.py
COPY scripts/deploy/vendor/gazebo-arm64/ /tmp/gazebo-arm64/
RUN cd /tmp/gazebo-arm64 && sha256sum -c SHA256SUMS
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl unzip nginx locales python3-pip python3-venv \
    python3-numpy python3-yaml python3-pil python3-pytest python3-requests \
    python3-rosdep python3-colcon-common-extensions \
    libgl1-mesa-dri libegl1-mesa mesa-utils xvfb xauth x11-utils \
    procps psmisc iproute2 ros-humble-desktop /tmp/gazebo-arm64/*.deb \
    ros-humble-moveit ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-rmw-cyclonedds-cpp && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
WORKDIR /opt/work01
COPY . .
# 将缺失的 ROS Gazebo 插件与项目放入同一工作空间，启动时统一 source install。
RUN mkdir -p third_party/gazebo_ros_pkgs third_party/gazebo_ros2_control \
    && tar -xzf /tmp/gazebo-arm64/gazebo_ros_pkgs.tar.gz --strip-components=1 -C third_party/gazebo_ros_pkgs \
    && tar -xzf /tmp/gazebo-arm64/gazebo_ros2_control.tar.gz --strip-components=1 -C third_party/gazebo_ros2_control \
    && touch third_party/gazebo_ros2_control/gazebo_ros2_control_demos/COLCON_IGNORE \
    && rm -rf /tmp/gazebo-arm64
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/python -m pip install --no-cache-dir -r jiang/requirements.txt
# 官方 ARM64 发布包随项目分发，目标机器无需访问 GitHub。
RUN test "$(uname -m)" = aarch64 \
    && cd scripts/deploy/vendor && sha256sum -c SHA256SUMS && cd /opt/work01 \
    && unzip -q scripts/deploy/vendor/zenoh-1.9.0-aarch64.zip -d /tmp/zenoh \
    && mkdir -p /opt/zenoh-bridge-ros2dds \
    && install -m 755 /tmp/zenoh/zenoh-bridge-ros2dds /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds \
    && /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds --version \
    && rm -rf /tmp/zenoh
RUN source /opt/ros/humble/setup.bash \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths xczs_inspection_robot_* third_party --ignore-src --rosdistro humble -y \
    && CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" MAKEFLAGS="-j$BUILD_JOBS" colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF \
    && source install/setup.bash \
    && ros2 pkg prefix gazebo_ros && ros2 pkg prefix gazebo_plugins \
    && ros2 pkg prefix gazebo_ros2_control && gzserver --version \
    && rm -rf /var/lib/apt/lists/*
RUN /opt/venv/bin/python scripts/deploy/prepare_assets.py \
    && cp scripts/deploy/nginx.conf /etc/nginx/sites-available/default \
    && nginx -t && chmod +x run_all.sh jiang/start_xczs_bridge.sh scripts/deploy/entrypoint.sh
ENV PYTHON_BIN=/opt/venv/bin/python ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CONTROL_HOST=127.0.0.1 CONTROL_PORT=8090 \
    LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
EXPOSE 80
HEALTHCHECK --start-period=600s --interval=15s --timeout=8s --retries=4 \
    CMD /opt/venv/bin/python /opt/work01/scripts/deploy/healthcheck.py
ENTRYPOINT ["/opt/work01/scripts/deploy/entrypoint.sh"]
