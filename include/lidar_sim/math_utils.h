#ifndef LIDAR_SIM_MATH_UTILS_H
#define LIDAR_SIM_MATH_UTILS_H

#include <cmath>

#include <Eigen/Geometry>

namespace lidar_sim {

inline double degToRad(double degrees) { return degrees * M_PI / 180.0; }

/// Rotation from roll, pitch, yaw [rad] using the Z-Y-X convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
inline Eigen::Quaterniond quaternionFromRpy(double roll, double pitch, double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

inline Eigen::Quaterniond quaternionFromRpy(const Eigen::Vector3d& rpy) {
  return quaternionFromRpy(rpy.x(), rpy.y(), rpy.z());
}

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MATH_UTILS_H
