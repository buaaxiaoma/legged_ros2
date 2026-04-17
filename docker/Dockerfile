FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-lc"]

ARG DEBIAN_FRONTEND=noninteractive
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG ALL_PROXY
ARG http_proxy
ARG https_proxy
ARG all_proxy

ARG UNITREE_ROS2_REPO=https://github.com/unitreerobotics/unitree_ros2.git
ARG UNITREE_ROS2_REF=master

ENV ROS_DISTRO=humble

RUN set -exo pipefail \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        git \
        libyaml-cpp-dev \
        pkg-config \
        python3-colcon-common-extensions \
        python3-rosdep \
        wget \
        ros-humble-ament-cmake \
        ros-humble-imu-tools \
        ros-humble-xacro \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

RUN set -exo pipefail \
    && rosdep init || true \
    && success=0 \
    && for i in {1..5}; do rosdep update && success=1 && break; sleep 10; done \
    && test "$success" -eq 1

# Build unitree_ros2 in its own workspace (repo internal cyclonedds_ws).
RUN set -exo pipefail \
    && success=0 \
    && for i in {1..5}; do git clone --branch "${UNITREE_ROS2_REF}" --depth 1 "${UNITREE_ROS2_REPO}" /root/unitree_ros2 && success=1 && break; rm -rf /root/unitree_ros2; sleep 10; done \
    && test "$success" -eq 1 \
    && sed -i 's|source /opt/ros/foxy/setup.bash|source /opt/ros/humble/setup.bash|g' /root/unitree_ros2/setup.sh \
    && sed -i 's|source /opt/ros/foxy/setup.bash|source /opt/ros/humble/setup.bash|g' /root/unitree_ros2/setup_local.sh \
    && source /root/unitree_ros2/setup_local.sh \
    && cd /root/unitree_ros2/cyclonedds_ws \
    && success=0 \
    && for i in {1..5}; do rosdep install --from-paths src --ignore-src -r -y && success=1 && break; sleep 10; done \
    && test "$success" -eq 1 \
    && colcon build --symlink-install

RUN set -exo pipefail \
    && mkdir -p /root/legged_ws/src

COPY docker/setup.sh /root/legged_ws/setup.sh
COPY docker/setup_local.sh /root/legged_ws/setup_local.sh
RUN chmod +x /root/legged_ws/setup.sh /root/legged_ws/setup_local.sh

WORKDIR /root/legged_ws
ENTRYPOINT ["/bin/bash"]
