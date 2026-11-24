cmake_minimum_required(VERSION 3.16)
project(lidar_sim)

# Two ways to build:
#   catkin (ROS Noetic) — nodes and, when Qt is installed, the viewer
#   plain CMake on Ubuntu 22.04 / 24.04 — the viewer only, no ROS
# Pass -DLIDAR_SIM_WITH_ROS=OFF to skip the nodes even if a ROS workspace is sourced.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin QUIET COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)
if(NOT DEFINED LIDAR_SIM_WITH_ROS)
  if(catkin_FOUND)
    set(LIDAR_SIM_WITH_ROS ON)
  else()
    set(LIDAR_SIM_WITH_ROS OFF)
  endif()
endif()
if(LIDAR_SIM_WITH_ROS AND NOT catkin_FOUND)
  message(FATAL_ERROR
    "LIDAR_SIM_WITH_ROS=ON but catkin was not found. Source a ROS setup.bash, or configure with -DLIDAR_SIM_WITH_ROS=OFF.")
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if(LIDAR_SIM_WITH_ROS)
  message(STATUS "lidar_sim: building ROS nodes")
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lidar_sim_core lidar_sim_ros
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf2_ros visualization_msgs
    DEPENDS EIGEN3 YAML_CPP
  )
else()
  message(STATUS "lidar_sim: building without ROS")
endif()

set(LIDAR_SIM_CORE_SOURCES
  src/geometry/plane_constraint.cpp
  src/geometry/primitive.cpp
  src/geometry/plane.cpp
  src/geometry/sphere.cpp
  src/geometry/cylinder.cpp
  src/geometry/ellipsoid.cpp
  src/world.cpp
  src/lidar_scanner.cpp
  src/sim_config.cpp
  src/motion/rigid_body.cpp
  src/motion/circular_trajectory.cpp
  src/motion/motion_simulator.cpp
  src/motion/waypoint_mission.cpp
  src/motion/quadrotor_controller.cpp
  src/motion/drone_simulator.cpp
)

add_library(lidar_sim_core ${LIDAR_SIM_CORE_SOURCES})
target_include_directories(lidar_sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(lidar_sim_core PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_directories(lidar_sim_core PUBLIC ${YAML_CPP_LIBRARY_DIRS})
target_link_libraries(lidar_sim_core PUBLIC ${YAML_CPP_LIBRARIES} Threads::Threads)
if(TARGET Eigen3::Eigen)
  target_link_libraries(lidar_sim_core PUBLIC Eigen3::Eigen)
else()
  target_include_directories(lidar_sim_core PUBLIC ${EIGEN3_INCLUDE_DIRS})
endif()

if(LIDAR_SIM_WITH_ROS)
  add_library(lidar_sim_ros
    src/ros/params.cpp
    src/ros/conversions.cpp
    src/ros/lidar_state_publisher.cpp
    src/ros/lidar_sim_node.cpp
    src/ros/motion_sim_node.cpp
    src/ros/drone_sim_node.cpp
  )
  target_include_directories(lidar_sim_ros PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(lidar_sim_ros PUBLIC lidar_sim_core ${catkin_LIBRARIES})
  add_dependencies(lidar_sim_ros ${catkin_EXPORTED_TARGETS})

  set(NODES sim_lidar sim_motion sim_drone investigate)
  foreach(node ${NODES})
    add_executable(${node} src/nodes/${node}.cpp)
    target_link_libraries(${node} lidar_sim_ros)
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
endif()

option(LIDAR_SIM_VIEWER "Build the Qt viewer" ON)
set(LIDAR_SIM_QT_VERSION "" CACHE STRING "Qt major version for the viewer (5 or 6). Empty prefers Qt 6.")
if(LIDAR_SIM_VIEWER)
  if(LIDAR_SIM_QT_VERSION STREQUAL "5")
    find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
    set(QT_VERSION_MAJOR 5)
    set(QT_FOUND TRUE)
  elseif(LIDAR_SIM_QT_VERSION STREQUAL "6")
    find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)
    set(QT_VERSION_MAJOR 6)
    set(QT_FOUND TRUE)
  else()
    find_package(QT NAMES Qt6 Qt5 QUIET COMPONENTS Widgets)
    if(NOT QT_FOUND)
      find_package(Qt5 QUIET COMPONENTS Widgets OpenGL)
      if(Qt5_FOUND)
        set(QT_FOUND TRUE)
        set(QT_VERSION_MAJOR 5)
      endif()
    endif()
    if(QT_FOUND AND QT_VERSION_MAJOR EQUAL 6)
      find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)
    elseif(QT_FOUND)
      find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
    endif()
  endif()
  if(NOT QT_FOUND AND LIDAR_SIM_WITH_ROS)
    message(STATUS "Qt not found; skipping lidar_sim_viewer")
  elseif(NOT QT_FOUND)
    message(FATAL_ERROR
      "Qt5 or Qt6 is required for the viewer.\n"
      "  Ubuntu 22.04 / 24.04: sudo apt install qt6-base-dev libgl1-mesa-dev libeigen3-dev libyaml-cpp-dev pkg-config")
  endif()

  if(QT_FOUND)
    message(STATUS "lidar_sim viewer: Qt${QT_VERSION_MAJOR}")
    add_executable(lidar_sim_viewer
      src/viewer/main.cpp
      src/viewer/main_window.cpp
      src/viewer/scene_view.cpp
      src/viewer/scene_mesh.cpp
      src/viewer/simulation_session.cpp
    )
    target_compile_definitions(lidar_sim_viewer PRIVATE LIDAR_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(lidar_sim_viewer PRIVATE lidar_sim_core)
    if(QT_VERSION_MAJOR EQUAL 6)
      target_link_libraries(lidar_sim_viewer PRIVATE Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
    else()
      target_link_libraries(lidar_sim_viewer PRIVATE Qt5::Widgets Qt5::OpenGL)
    endif()
    if(LIDAR_SIM_WITH_ROS)
      install(TARGETS lidar_sim_viewer RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    endif()
  endif()
endif()

if(NOT LIDAR_SIM_WITH_ROS AND TARGET lidar_sim_viewer)
  include(GNUInstallDirs)
  install(TARGETS lidar_sim_viewer lidar_sim_core
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
  install(DIRECTORY include/lidar_sim DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/lidar_sim)
endif()
