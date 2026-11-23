#ifndef LIDAR_SIM_MOTION_MOTION_SIMULATOR_H
#define LIDAR_SIM_MOTION_MOTION_SIMULATOR_H

#include <Eigen/Geometry>

#include "lidar_sim/motion/circular_trajectory.h"
#include "lidar_sim/motion/rigid_body.h"

namespace lidar_sim {

/// Kinematic body (no attitude dynamics) that tracks a circular trajectory with a
/// position/velocity feedback law. The body stays still during an initial delay.
class MotionSimulator {
 public:
  struct Config {
    double gravity = 9.81;       ///< [m/s^2]
    double start_delay = 0.0;    ///< Time the body stays still before tracking starts [s].
    double position_gain = 0.0;  ///< Velocity correction per meter of position error [1/s].
    double velocity_gain = 0.0;  ///< Acceleration per m/s of velocity error [1/s].
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond initial_orientation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond lidar_mount = Eigen::Quaterniond::Identity();  ///< LiDAR to body.
    CircularTrajectory::Config trajectory;
  };

  explicit MotionSimulator(const Config& config);

  /// Advances the simulation by `dt`, with `t` the time elapsed since the start.
  void step(double t, double dt);

  const RigidBodyState& state() const { return state_; }
  const ImuMeasurement& imu() const { return imu_; }

 private:
  Config config_;
  CircularTrajectory trajectory_;
  RigidBodyState state_;
  ImuMeasurement imu_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MOTION_MOTION_SIMULATOR_H
