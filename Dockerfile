# syntax=docker/dockerfile:1.4
FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and rosdep (with apt cache mount for faster rebuilds)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    apt install -y --no-install-recommends \
    xserver-xorg \
    python3-pip

# Install MoveIt and required packages (with apt cache mount)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-gazebo-msgs \
    ros-humble-cv-bridge \
    ros-humble-angles \
    ros-humble-image-geometry \
    ros-humble-pcl-ros \
    ros-humble-depth-image-proc \
    libopencv-dev \
    python3-open3d \
    python3-numpy \
    python3-sklearn \
    python3-matplotlib

# rosdep update (once)
RUN rosdep update

# Install dependencies (with apt cache mount)
RUN mkdir -p /tmp/ros2_dependencies/src
COPY ros2/src /tmp/ros2_dependencies/src
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    cd /tmp/ros2_dependencies && \
    rosdep install -r -y -i --from-paths . && \
    rm -rf /tmp/ros2_dependencies

RUN mkdir /workspace
WORKDIR /workspace

# Build ROS2 workspace
FROM base AS built
COPY ros2 /workspace/ros2
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace/ros2 && \
    colcon build --symlink-install && \
    rm -rf log

FROM base AS dev

RUN apt-get update && apt install -y --no-install-recommends \
    vim \
    tmux \
    x11-apps

CMD ["/bin/bash"]
