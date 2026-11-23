#include "lidar_sim/motion/drone_simulator.h"

namespace lidar_sim {

DroneSimulator::DroneSimulator(const Config& config)
    : config_(config), mission_(config.waypoints), controller_(config.controller) {
  state_.position = config_.initial_position;
  state_.orientation = config_.initial_orientation.normalized();
  target_ = &mission_.activeWaypoint(0.0);
  imu_ = measureImu(state_, state_.orientation * Eigen::Vector3d(0.0, 0.0, config_.gravity),
                    config_.lidar_mount);
}

void DroneSimulator::step(double t, double dt) {
  target_ = &mission_.activeWaypoint(t);

  QuadrotorCommand command = controller_.compute(state_, *target_);
  if (t < config_.hover_duration) {
    command.thrust = config_.gravity;
    command.body_rates.setZero();
  }
  if (state_.position.z() < config_.min_attitude_control_altitude) {
    command.body_rates.setZero();
  }

  const Eigen::Vector3d thrust_acceleration =
      state_.orientation * Eigen::Vector3d(0.0, 0.0, command.thrust);
  const Eigen::Vector3d acceleration = thrust_acceleration +
                                       Eigen::Vector3d(0.0, 0.0, -config_.gravity) -
                                       config_.drag_coefficient * state_.velocity;

  state_.angular_velocity = command.body_rates;
  state_.integrate(acceleration, dt);
  imu_ = measureImu(state_, thrust_acceleration, config_.lidar_mount);
}

}  // namespace lidar_sim
