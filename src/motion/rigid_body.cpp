#include "lidar_sim/motion/rigid_body.h"

namespace lidar_sim {

void RigidBodyState::integrate(const Eigen::Vector3d& acceleration, double dt) {
  position += velocity * dt;
  velocity += acceleration * dt;

  // q_dot = 0.5 * q * (0, w), with w in the body frame.
  const Eigen::Quaterniond omega(0.0, angular_velocity.x(), angular_velocity.y(),
                                 angular_velocity.z());
  const Eigen::Quaterniond q_dot = orientation * omega;
  orientation.coeffs() += 0.5 * dt * q_dot.coeffs();
  orientation.normalize();
}

ImuMeasurement measureImu(const RigidBodyState& body, const Eigen::Vector3d& specific_force_world,
                          const Eigen::Quaterniond& mount) {
  ImuMeasurement imu;
  imu.orientation = body.orientation * mount;
  imu.angular_velocity = mount.conjugate() * body.angular_velocity;
  imu.linear_acceleration = imu.orientation.conjugate() * specific_force_world;
  return imu;
}

}  // namespace lidar_sim
