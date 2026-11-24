#ifndef LIDAR_SIM_MOTION_CIRCULAR_TRAJECTORY_H
#define LIDAR_SIM_MOTION_CIRCULAR_TRAJECTORY_H

#include <Eigen/Core>

namespace lidar_sim {

/// Right-handed unit axes of the plane with this normal: first × second points along `normal`.
/// A normal of (0, 0, 1) gives first = (1, 0, 0) and second = (0, 1, 0), a circle parallel to xy.
/// @throws std::invalid_argument if `normal` has zero length.
void circleAxesFromNormal(const Eigen::Vector3d& normal, Eigen::Vector3d& first_axis,
                          Eigen::Vector3d& second_axis);

/// Angular speed [rad/s] for a tangential speed [m/s]. Returns 0 when `radius` is 0.
/// Positive speed follows the right-hand rule about the plane normal.
double angularSpeedFromLinear(double linear_speed, double radius);

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
