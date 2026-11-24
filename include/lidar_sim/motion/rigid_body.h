#ifndef LIDAR_SIM_MOTION_RIGID_BODY_H
#define LIDAR_SIM_MOTION_RIGID_BODY_H

#include <Eigen/Geometry>

namespace lidar_sim {

/// Kinematic state of a rigid body.
struct RigidBodyState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();          ///< World frame [m].
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();          ///< World frame [m/s].
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  ///< Body to world.
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  ///< Body frame [rad/s].

  /// Advances the state by one explicit Euler step, holding the world-frame `acceleration` and
  /// the current body angular velocity constant over `dt`.
  void integrate(const Eigen::Vector3d& acceleration, double dt);
};

/// Reading of an IMU rigidly attached to a body.
struct ImuMeasurement {
  Eigen::Quaterniond orientation;       ///< IMU to world.
  Eigen::Vector3d angular_velocity;     ///< IMU frame [rad/s].
  Eigen::Vector3d linear_acceleration;  ///< Specific force in the IMU frame [m/s^2].
};

/// Ideal IMU mounted on the body with the rotation `mount` (IMU to body), at the body origin.
/// @param specific_force_world Non-gravitational acceleration of the body, in the world frame.
ImuMeasurement measureImu(const RigidBodyState& body, const Eigen::Vector3d& specific_force_world,
                          const Eigen::Quaterniond& mount);

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MOTION_RIGID_BODY_H
