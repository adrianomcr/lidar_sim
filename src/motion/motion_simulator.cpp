#include "lidar_sim/motion/motion_simulator.h"

namespace lidar_sim {

MotionSimulator::MotionSimulator(const Config& config)
    : config_(config), trajectory_(config.trajectory) {
  state_.position = config_.initial_position;
  state_.orientation = config_.initial_orientation.normalized();
  imu_ = measureImu(state_, Eigen::Vector3d(0.0, 0.0, config_.gravity), config_.lidar_mount);
}

void MotionSimulator::step(double t, double dt) {
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  if (t >= config_.start_delay) {
    const Eigen::Vector3d velocity_command =
        trajectory_.velocity(t) +
        config_.position_gain * (trajectory_.position(t) - state_.position);
    acceleration = config_.velocity_gain * (velocity_command - state_.velocity);
  }

  state_.integrate(acceleration, dt);

  // The IMU feels the commanded acceleration plus the reaction to gravity.
  const Eigen::Vector3d specific_force = acceleration + Eigen::Vector3d(0.0, 0.0, config_.gravity);
  imu_ = measureImu(state_, specific_force, config_.lidar_mount);
}

}  // namespace lidar_sim
