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
# 将缺失的 ROS Gazebo 插件与项目放入同一工作空间，启动时统一 source install。
RUN mkdir -p third_party/gazebo_ros_pkgs third_party/gazebo_ros2_control \
    && tar -xzf /tmp/gazebo-arm64/gazebo_ros_pkgs.tar.gz --strip-components=1 -C third_party/gazebo_ros_pkgs \
    && tar -xzf /tmp/gazebo-arm64/gazebo_ros2_control.tar.gz --strip-components=1 -C third_party/gazebo_ros2_control \
    && touch third_party/gazebo_ros2_control/gazebo_ros2_control_demos/COLCON_IGNORE \
    && rm -rf /tmp/gazebo-arm64
COPY jiang/requirements.txt jiang/requirements.txt
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/python -m pip install --no-cache-dir -r jiang/requirements.txt \
    && /opt/venv/bin/python -c 'from passlib.context import CryptContext; c=CryptContext(schemes=["bcrypt_sha256"]); s="deployment-check-" * 8; h=c.hash(s); assert c.verify(s,h); assert not c.verify(s+"wrong",h); print("Password backend check passed")'
# 官方 ARM64 发布包随项目分发，目标机器无需访问 GitHub。
COPY scripts/deploy/vendor/zenoh-1.9.0-aarch64.zip scripts/deploy/vendor/SHA256SUMS scripts/deploy/vendor/
RUN test "$(uname -m)" = aarch64 \
    && cd scripts/deploy/vendor && sha256sum -c SHA256SUMS && cd /opt/work01 \
    && unzip -q scripts/deploy/vendor/zenoh-1.9.0-aarch64.zip -d /tmp/zenoh \
    && mkdir -p /opt/zenoh-bridge-ros2dds \
    && install -m 755 /tmp/zenoh/zenoh-bridge-ros2dds /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds \
    && /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds --version \
    && rm -rf /tmp/zenoh
# 网络依赖与项目源码分层：源码/版本检查修改不再触发 rosdep update。
COPY scripts/deploy/update_rosdep.py /usr/local/lib/xczs/update_rosdep.py
RUN printf 'Acquire::ForceIPv4 "true";\nAcquire::Retries "5";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' > /etc/apt/apt.conf.d/80-xczs-network \
    && python3 /usr/local/lib/xczs/update_rosdep.py
# start 只提取 package.xml；依赖变化才使这一层失效。
COPY .deploy-manifests/ /tmp/xczs-manifests/
RUN source /opt/ros/humble/setup.bash \
    && apt-get update -o APT::Update::Error-Mode=any \
    && rosdep install --from-paths /tmp/xczs-manifests third_party --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/* /tmp/xczs-manifests
# ARM64 发行包清空了 SingletonT 的共享声明；必须重建本体及其使用方。
# 保留此前下载层；原生依赖、源码编译、实际数据验收各自缓存。
RUN apt-get update -o APT::Update::Error-Mode=any \
    && apt-get install -y --no-install-recommends \
    patch binutils gzip pkg-config fonts-roboto-unhinted fonts-dejavu-core fonts-liberation \
    protobuf-compiler libprotoc-dev libprotobuf-dev \
    libgraphviz-dev libccd-dev libopenal-dev libfreeimage-dev freeglut3-dev \
    libcurl4-openssl-dev libtinyxml-dev libtinyxml2-dev libtar-dev libtbb-dev \
    libogre-1.9-dev libxml2-dev qtbase5-dev libqwt-qt5-dev libqt5opengl5-dev \
    libboost-dev libboost-thread-dev libboost-system-dev libboost-filesystem-dev \
    libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev libboost-date-time-dev \
    libgts-dev libavformat-dev libavcodec-dev libswscale-dev libusb-1.0-0-dev libspnav-dev \
    libsdformat9-dev libignition-math6-dev libignition-transport8-dev libignition-msgs5-dev \
    libignition-common3-dev libignition-fuel-tools4-dev libbullet-dev libsimbody-dev \
    libgdal-dev libdart-dev libdart-collision-bullet-dev libdart-collision-ode-dev \
    libdart-utils-urdf-dev libdart-external-convhull-3d-dev \
    libdart-external-ikfast-dev libdart-external-imgui-dev libdart-external-lodepng-dev \
    libdart-external-odelcpsolver-dev libgtest-dev libtinyobjloader-dev \
    && rm -rf /var/lib/apt/lists/*
COPY scripts/deploy/vendor/gazebo-source/ /tmp/gazebo-source/
COPY scripts/deploy/build_gazebo_core.sh scripts/deploy/check_gazebo_singletons.py /usr/local/lib/xczs/
RUN bash /usr/local/lib/xczs/build_gazebo_core.sh /tmp/gazebo-source && rm -rf /tmp/gazebo-source
ENV PATH=/opt/gazebo-fixed/bin:${PATH} \
    LD_LIBRARY_PATH=/opt/gazebo-fixed/lib \
    CMAKE_PREFIX_PATH=/opt/gazebo-fixed \
    PKG_CONFIG_PATH=/opt/gazebo-fixed/lib/pkgconfig \
    GAZEBO_MODEL_PATH=/opt/gazebo-fixed/share/gazebo-11/models \
    GAZEBO_RESOURCE_PATH=/opt/gazebo-fixed/share/gazebo-11 \
    GAZEBO_PLUGIN_PATH=/opt/gazebo-fixed/lib/gazebo-11/plugins
# docker exec bash -lc 会重设 PATH；诊断和交互终端也必须使用新本体。
RUN printf '%s\n' 'export PATH=/opt/gazebo-fixed/bin:$PATH' \
    'export LD_LIBRARY_PATH=/opt/gazebo-fixed/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}' \
    > /etc/profile.d/xczs-gazebo.sh
COPY scripts/deploy/gazebo_native_probe.py /usr/local/lib/xczs/
RUN test "$(command -v gzserver)" = /opt/gazebo-fixed/bin/gzserver \
    && bash -lc 'test "$(command -v gzserver)" = /opt/gazebo-fixed/bin/gzserver' \
    && ldd /opt/gazebo-fixed/bin/gzserver > /tmp/gazebo-linkage.txt \
    && cat /tmp/gazebo-linkage.txt \
    && ! grep -q 'not found' /tmp/gazebo-linkage.txt \
    && ! grep -E 'libgazebo[^ ]* => ' /tmp/gazebo-linkage.txt | grep -v '=> /opt/gazebo-fixed/lib/' \
    && python3 -u /usr/local/lib/xczs/gazebo_native_probe.py
COPY xczs_inspection_robot_bringup/ xczs_inspection_robot_bringup/
COPY xczs_inspection_robot_control/ xczs_inspection_robot_control/
COPY xczs_inspection_robot_description/ xczs_inspection_robot_description/
COPY xczs_inspection_robot_gazebo/ xczs_inspection_robot_gazebo/
COPY xczs_inspection_robot_interfaces/ xczs_inspection_robot_interfaces/
COPY xczs_inspection_robot_moveit_config/ xczs_inspection_robot_moveit_config/
COPY xczs_inspection_robot_nav2/ xczs_inspection_robot_nav2/
RUN source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" MAKEFLAGS="-j$BUILD_JOBS" colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
# 与编译分层；版本命令的特殊退出码不再使成功编译丢失。
RUN source /opt/ros/humble/setup.bash && source install/setup.bash \
    && ros2 pkg prefix gazebo_ros && ros2 pkg prefix gazebo_plugins \
    && ros2 pkg prefix gazebo_ros2_control \
    && (status=0; timeout 20 gzserver --version > /tmp/gazebo-version.txt 2>&1 || status=$?; \
        cat /tmp/gazebo-version.txt; \
        { [ "$status" -eq 0 ] || [ "$status" -eq 255 ]; } \
        && grep -q 'version 11\.' /tmp/gazebo-version.txt)
COPY . .
RUN rm -rf .deploy-manifests && /opt/venv/bin/python scripts/deploy/prepare_assets.py \
    && cp scripts/deploy/nginx.conf /etc/nginx/sites-available/default \
    && nginx -t && chmod +x run_all.sh jiang/start_xczs_bridge.sh scripts/deploy/entrypoint.sh
ENV PYTHON_BIN=/opt/venv/bin/python ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CONTROL_HOST=127.0.0.1 CONTROL_PORT=8090 \
    LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
EXPOSE 80
HEALTHCHECK --start-period=600s --interval=15s --timeout=8s --retries=4 \
    CMD /opt/venv/bin/python /opt/work01/scripts/deploy/healthcheck.py
ENTRYPOINT ["/opt/work01/scripts/deploy/entrypoint.sh"]
