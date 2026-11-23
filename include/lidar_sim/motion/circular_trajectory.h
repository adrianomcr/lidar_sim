#ifndef LIDAR_SIM_MOTION_CIRCULAR_TRAJECTORY_H
#define LIDAR_SIM_MOTION_CIRCULAR_TRAJECTORY_H

#include <Eigen/Core>

namespace lidar_sim {

/// Constant-speed circle: p(t) = center + radius * (cos(w t) * first_axis + sin(w t) * second_axis).
class CircularTrajectory {
 public:
  struct Config {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double radius = 1.0;                                   ///< [m]
    double angular_speed = 0.0;                            ///< w [rad/s]
    Eigen::Vector3d first_axis = Eigen::Vector3d::UnitX();  ///< Direction of p(0) from the center.
    Eigen::Vector3d second_axis = Eigen::Vector3d::UnitY();
  };

  /// @throws std::invalid_argument if the axes are not orthogonal or have zero length.
  explicit CircularTrajectory(const Config& config);

  Eigen::Vector3d position(double t) const;
  Eigen::Vector3d velocity(double t) const;

 private:
  Config config_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MOTION_CIRCULAR_TRAJECTORY_H
