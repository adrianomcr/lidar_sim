#!/usr/bin/env bash
# Rebuild lidar_sim from this directory.
#
#   ./build.sh
#       Standalone viewer, no ROS. Then, still in this directory:
#         ./build/lidar_sim_viewer
#
#   ./build.sh ros
#       ROS Noetic package (nodes, and the viewer when Qt is installed).
#       Run from a sourced catkin workspace.
#
# Viewer packages on Ubuntu 22.04 / 24.04:
#   sudo apt install build-essential cmake pkg-config libeigen3-dev \
#     libyaml-cpp-dev qt6-base-dev libgl1-mesa-dev

set -euo pipefail
cd "$(dirname "$0")"

if [[ "${1:-}" == "ros" ]]; then
  echo "Building the ROS package with catkin..."
  catkin build lidar_sim --cmake-args -DCMAKE_BUILD_TYPE=Release
  exit 0
fi

if [[ $# -gt 0 ]]; then
  echo "Unknown option: $1" >&2
  echo "Usage: ./build.sh [ros]" >&2
  exit 2
fi

echo "Building the standalone viewer..."
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DLIDAR_SIM_WITH_ROS=OFF
cmake --build build -j"$(nproc)"

echo
echo "Built ./build/lidar_sim_viewer"
echo "Run it from this directory so it opens config/params.yaml:"
echo "  ./build/lidar_sim_viewer"
