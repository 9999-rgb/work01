FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl unzip nginx locales python3-pip python3-venv \
    python3-numpy python3-yaml python3-pil python3-pytest python3-requests \
    python3-rosdep python3-colcon-common-extensions \
    libgl1-mesa-dri libegl1-mesa mesa-utils xvfb xauth x11-utils \
    procps psmisc iproute2 ros-humble-desktop ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control ros-humble-moveit ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-rmw-cyclonedds-cpp && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
WORKDIR /opt/work01
COPY . .
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/python -m pip install --no-cache-dir -r jiang/requirements.txt
RUN arch="$(uname -m)" && case "$arch" in aarch64|x86_64) ;; *) exit 1;; esac \
    && curl -fL --retry 3 "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.9.0/zenoh-plugin-ros2dds-1.9.0-${arch}-unknown-linux-gnu-standalone.zip" -o /tmp/zenoh.zip \
    && unzip -q /tmp/zenoh.zip -d /tmp/zenoh \
    && mkdir -p /opt/zenoh-bridge-ros2dds \
    && install -m 755 "$(find /tmp/zenoh -type f -name zenoh-bridge-ros2dds -print -quit)" /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds \
    && /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds --version \
    && rm -rf /tmp/zenoh /tmp/zenoh.zip
RUN source /opt/ros/humble/setup.bash \
    && { test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init; } \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths xczs_inspection_robot_* --ignore-src --rosdistro humble -y \
    && CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" MAKEFLAGS="-j$BUILD_JOBS" colcon build --symlink-install --executor sequential \
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
