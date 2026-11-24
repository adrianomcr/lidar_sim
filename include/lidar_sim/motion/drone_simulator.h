#ifndef LIDAR_SIM_MOTION_DRONE_SIMULATOR_H
#define LIDAR_SIM_MOTION_DRONE_SIMULATOR_H

#include <vector>

#include <Eigen/Geometry>

#include "lidar_sim/motion/quadrotor_controller.h"
#include "lidar_sim/motion/rigid_body.h"
#include "lidar_sim/motion/waypoint_mission.h"

namespace lidar_sim {

/// Quadrotor carrying a LiDAR, flying a waypoint mission.
///
/// The drone hovers in place for `hover_duration`, then follows the mission. Below
/// `min_attitude_control_altitude` the attitude is frozen, so it climbs straight up first.
/// Dynamics: thrust along body z, gravity and linear drag; body rates are tracked perfectly.
class DroneSimulator {
 public:
  struct Config {
    double gravity = 9.81;                      ///< [m/s^2]
    double drag_coefficient = 0.0;              ///< Linear drag per unit mass [1/s].
    double hover_duration = 0.0;                ///< [s]
    double min_attitude_control_altitude = 0.0;  ///< [m]
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond initial_orientation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond lidar_mount = Eigen::Quaterniond::Identity();  ///< LiDAR to body.
    QuadrotorController::Config controller;
    std::vector<Waypoint> waypoints;
  };

  explicit DroneSimulator(const Config& config);

  /// Advances the simulation by `dt`, with `t` the time elapsed since the start.
  void step(double t, double dt);

  const RigidBodyState& state() const { return state_; }
  const ImuMeasurement& imu() const { return imu_; }
  const Waypoint& target() const { return *target_; }

 private:
  Config config_;
  WaypointMission mission_;
  QuadrotorController controller_;
  RigidBodyState state_;
  ImuMeasurement imu_;
  const Waypoint* target_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MOTION_DRONE_SIMULATOR_H
