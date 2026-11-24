cmake_minimum_required(VERSION 3.10)
project(lidar_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ray casting is far too slow without optimizations.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_sim_core lidar_sim_ros
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf2_ros visualization_msgs
  DEPENDS EIGEN3 YAML_CPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# ROS-independent simulation library: geometry, world, LiDAR model and motion models.
add_library(lidar_sim_core
  src/geometry/plane_constraint.cpp
  src/geometry/primitive.cpp
  src/geometry/plane.cpp
  src/geometry/sphere.cpp
  src/geometry/cylinder.cpp
  src/geometry/ellipsoid.cpp
  src/world.cpp
  src/lidar_scanner.cpp
  src/motion/rigid_body.cpp
  src/motion/circular_trajectory.cpp
  src/motion/motion_simulator.cpp
  src/motion/waypoint_mission.cpp
  src/motion/quadrotor_controller.cpp
  src/motion/drone_simulator.cpp
)
target_link_libraries(lidar_sim_core ${YAML_CPP_LIBRARIES} Threads::Threads)

# ROS front ends of the simulators.
add_library(lidar_sim_ros
  src/ros/params.cpp
  src/ros/conversions.cpp
  src/ros/lidar_state_publisher.cpp
  src/ros/lidar_sim_node.cpp
  src/ros/motion_sim_node.cpp
  src/ros/drone_sim_node.cpp
)
target_link_libraries(lidar_sim_ros lidar_sim_core ${catkin_LIBRARIES})
add_dependencies(lidar_sim_ros ${catkin_EXPORTED_TARGETS})

set(NODES sim_lidar sim_motion sim_drone investigate)
foreach(node ${NODES})
  add_executable(${node} src/nodes/${node}.cpp)
  target_link_libraries(${node} lidar_sim_ros ${catkin_LIBRARIES})
endforeach()

install(TARGETS lidar_sim_core lidar_sim_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS ${NODES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/create_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
