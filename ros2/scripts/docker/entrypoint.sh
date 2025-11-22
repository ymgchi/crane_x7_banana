#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2025 ymgchi

set -e  # Exit on error

# Resolve script directory using realpath for absolute path resolution
SCRIPT_DIR=$(dirname $(realpath $0))
# Script is at /workspace/ros2/scripts/docker, go up 3 levels to /workspace
WORKSPACE_DIR=$(realpath $SCRIPT_DIR/../../..)
ROS2_WORKSPACE=$WORKSPACE_DIR/ros2

echo "=== ROS 2 Workspace Setup ==="
echo "SCRIPT_DIR: $SCRIPT_DIR"
echo "WORKSPACE_DIR: $WORKSPACE_DIR"
echo "ROS2_WORKSPACE: $ROS2_WORKSPACE"

if [ ! -d "$ROS2_WORKSPACE" ]; then
    echo "ERROR: ROS2_WORKSPACE directory not found: $ROS2_WORKSPACE"
    exit 1
fi

cd $ROS2_WORKSPACE

if [ ! -f "$ROS2_WORKSPACE/install/setup.bash" ]; then
    echo "ERROR: setup.bash not found: $ROS2_WORKSPACE/install/setup.bash"
    echo "Please build the workspace first using ros2_builder service."
    exit 1
fi

source $ROS2_WORKSPACE/install/setup.bash

echo "ROS 2 workspace ready!"
echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
echo "============================="
echo ""

# Check user groups
echo "=== User Groups Check ==="
echo "Current user: $(whoami) (UID: $(id -u), GID: $(id -g))"
echo "Groups: $(groups)"
echo ""

# Check GPU/DRI devices for Gazebo
echo "=== GPU/DRI Device Check ==="
if [ -d "/dev/dri" ]; then
    echo "Available DRI devices:"
    ls -la /dev/dri/ 2>/dev/null || echo "No /dev/dri devices found"
    echo ""
else
    echo "WARNING: /dev/dri directory not found"
    echo "Gazebo may not work properly without GPU access"
fi
echo "=========================="
echo ""

# Check if running in simulation mode
if [ "${SIMULATION_MODE:-false}" = "true" ]; then
    echo "=== Simulation Mode Enabled ==="
    echo "Starting Gazebo simulation environment..."
    echo ""
fi

exec "$@"
