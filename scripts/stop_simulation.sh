#!/usr/bin/env bash
# Stop all simulation and ROS2 nodes (Gazebo, launch, move_group, rviz2, etc.)
# Run from workspace root, or from inside the ur_arm container.

set -e

stop_procs() {
  for pattern in "$@"; do
    if pgrep -f "$pattern" >/dev/null 2>&1; then
      echo "Stopping: $pattern"
      pkill -f "$pattern" 2>/dev/null || true
    fi
  done
}

# Gazebo
stop_procs "gzserver" "gzclient"

# ROS2 launch (parent of all launched nodes)
stop_procs "ros2 launch" "launch_ros"

# Simulation / control nodes
stop_procs "spawn_entity" "spawner" "robot_state_publisher" "controller_manager"
stop_procs "move_group" "rviz2" "servo_node" "motion_control_node" "arpa_gui"
stop_procs "static_transform_publisher"

# Give processes a moment to exit
sleep 1

# Force-kill any remaining Gazebo or launch processes
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true

echo "Simulation and nodes stopped."
