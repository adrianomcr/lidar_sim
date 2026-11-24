#include "lidar_sim/motion/quadrotor_controller.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace lidar_sim {

QuadrotorCommand QuadrotorController::compute(const RigidBodyState& state,
                                              const Waypoint& target) const {
  // Velocity field towards the waypoint.
  const Eigen::Vector3d error = state.position - target.position;
  const double distance = error.norm();
  const double speed =
      config_.cruise_speed * (2.0 / M_PI) * std::atan(config_.distance_gain * distance);
  const Eigen::Vector3d velocity_reference = -speed * error / (distance + 1e-6);

  // Acceleration the thrust has to produce, gravity compensation included.
  const Eigen::Vector3d thrust_vector =
      config_.velocity_gain * (velocity_reference - state.velocity) +
      Eigen::Vector3d(0.0, 0.0, config_.gravity);

  // Reference attitude: body z along the thrust, body x as close as possible to the target yaw.
  const Eigen::Vector3d z_axis = thrust_vector.normalized();
  Eigen::Vector3d x_axis(std::cos(target.yaw), std::sin(target.yaw), 0.0);
  x_axis = (x_axis - x_axis.dot(z_axis) * z_axis).normalized();
  Eigen::Matrix3d reference_rotation;
  reference_rotation << x_axis, z_axis.cross(x_axis), z_axis;

  const Eigen::Quaterniond attitude_error =
      state.orientation.conjugate() * Eigen::Quaterniond(reference_rotation);
  const Eigen::AngleAxisd error_axis_angle(attitude_error);

  QuadrotorCommand command;
  command.thrust =
      std::min(thrust_vector.dot(z_axis), config_.max_thrust_to_weight * config_.gravity);
  command.body_rates =
      config_.attitude_gain * std::sin(error_axis_angle.angle()) * error_axis_angle.axis();
  command.body_rates.z() =
      std::max(-config_.max_yaw_rate, std::min(command.body_rates.z(), config_.max_yaw_rate));
  return command;
}

}  // namespace lidar_sim
