#!/bin/bash
# Run this *inside* Docker when build fails with:
#   "The current CMakeCache.txt directory ... is different than the directory
#    /home/the2xman/arpa_ws/build/... where CMakeCache.txt was created."
# That happens when build/ was created on the host and the same workspace
# is used in Docker at a different path (e.g. /root/ros2_ws).
set -e
WS="${1:-.}"
cd "$WS"
if [ -n "$2" ]; then
  # Clean only given package(s): ./clean_build_for_docker.sh . arpa_control arpa_behavior_trees
  for pkg in "${@:2}"; do
    echo "Removing build/$pkg install/$pkg"
    rm -rf "build/$pkg" "install/$pkg"
  done
else
  echo "Cleaning entire build, install, log so colcon reconfigures for this path..."
  rm -rf build install log
fi
echo "Done. Run: colcon build"
