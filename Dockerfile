FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive

# Update package lists and rosdep
RUN apt-get update && apt-get upgrade -y && \
    rosdep update && \
    apt install -y --no-install-recommends \
    xserver-xorg \
    python3-pip

# Install MoveIt and required packages
RUN apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-gazebo-msgs \
    ros-humble-cv-bridge \
    ros-humble-angles \
    ros-humble-image-geometry \
    ros-humble-pcl-ros \
    libopencv-dev

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

