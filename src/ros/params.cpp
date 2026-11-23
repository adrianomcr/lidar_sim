#include "lidar_sim/ros/params.h"

#include <vector>

#include "lidar_sim/math_utils.h"

namespace lidar_sim {

Eigen::Vector3d getVector3Param(const ros::NodeHandle& nh, const std::string& name) {
  const auto values = getRequiredParam<std::vector<double>>(nh, name);
  if (values.size() != 3) {
    throw std::runtime_error("Parameter '" + nh.resolveName(name) + "' must have 3 elements");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Quaterniond getRpyDegParam(const ros::NodeHandle& nh, const std::string& name) {
  const Eigen::Vector3d rpy_deg = getVector3Param(nh, name);
  return quaternionFromRpy(degToRad(rpy_deg.x()), degToRad(rpy_deg.y()), degToRad(rpy_deg.z()));
}

}  // namespace lidar_sim
