#ifndef LIDAR_SIM_ROS_PARAMS_H
#define LIDAR_SIM_ROS_PARAMS_H

#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <ros/node_handle.h>

namespace lidar_sim {

/// Reads a parameter that must exist on the parameter server.
/// @throws std::runtime_error naming the fully resolved parameter if it is missing or mistyped.
template <typename T>
T getRequiredParam(const ros::NodeHandle& nh, const std::string& name) {
  T value;
  if (!nh.getParam(name, value)) {
    throw std::runtime_error("Missing or invalid parameter '" + nh.resolveName(name) +
                             "'. Is config/params.yaml loaded?");
  }
  return value;
}

/// Reads a list of 3 numbers.
Eigen::Vector3d getVector3Param(const ros::NodeHandle& nh, const std::string& name);

/// Reads [roll, pitch, yaw] in degrees and returns the corresponding rotation.
Eigen::Quaterniond getRpyDegParam(const ros::NodeHandle& nh, const std::string& name);

}  // namespace lidar_sim

#endif  // LIDAR_SIM_ROS_PARAMS_H
