#!/usr/bin/env bash

echo "Compiling ..."
catkin build lidar_sim  --cmake-args -DCMAKE_BUILD_TYPE=Release
