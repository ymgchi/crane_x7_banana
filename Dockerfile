FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and rosdep
RUN apt-get update && apt-get upgrade -y && \
    rosdep update && \
    apt install -y --no-install-recommends \
    xserver-xorg \
    python3-pip

# Install MoveIt related packages
RUN apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-gazebo-msgs

# Install dependencies
RUN mkdir -p /tmp/ros2_dependencies/src
COPY ros2/src /tmp/ros2_dependencies/src
RUN cd /tmp/ros2_dependencies && \
    rosdep update && \
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

