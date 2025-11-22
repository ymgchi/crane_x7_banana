#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2025 ymgchi

set -e  # Exit on error

# Resolve script directory using realpath for absolute path resolution
SCRIPT_DIR=$(dirname $(realpath $0))
# Script is at /workspace/ros2/scripts/docker, go up 3 levels to /workspace
WORKSPACE_DIR=$(realpath $SCRIPT_DIR/../../..)
ROS2_WORKSPACE=$WORKSPACE_DIR/ros2

echo "=== ROS 2 Builder ==="
echo "SCRIPT_DIR: $SCRIPT_DIR"
echo "WORKSPACE_DIR: $WORKSPACE_DIR"
echo "ROS2_WORKSPACE: $ROS2_WORKSPACE"

if [ ! -d "$ROS2_WORKSPACE" ]; then
    echo "ERROR: ROS2_WORKSPACE directory not found: $ROS2_WORKSPACE"
    exit 1
fi

cd $ROS2_WORKSPACE

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

echo "Building ROS 2 workspace..."
echo "=========================="

# Build with symlink-install for faster development
colcon build --symlink-install

echo ""
echo "=== Build Complete ==="
echo "Workspace built successfully!"
echo "======================"
