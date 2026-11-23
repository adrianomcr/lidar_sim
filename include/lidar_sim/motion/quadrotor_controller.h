#ifndef LIDAR_SIM_MOTION_QUADROTOR_CONTROLLER_H
#define LIDAR_SIM_MOTION_QUADROTOR_CONTROLLER_H

#include <Eigen/Core>

#include "lidar_sim/motion/rigid_body.h"
#include "lidar_sim/motion/waypoint_mission.h"

namespace lidar_sim {

/// Collective thrust and body rates, the inputs of a quadrotor's attitude-rate loop.
struct QuadrotorCommand {
  double thrust = 0.0;  ///< Mass-normalized thrust along the body z axis [m/s^2].
  Eigen::Vector3d body_rates = Eigen::Vector3d::Zero();  ///< [rad/s]
};

/// Flies a quadrotor towards a waypoint.
///
/// A vector field commands a velocity pointing at the waypoint with speed
/// cruise_speed * (2 / pi) * atan(distance_gain * distance), which saturates far away and fades
/// out close to it. The resulting acceleration demand fixes the thrust direction, the waypoint yaw
/// fixes the heading, and the body rates rotate the vehicle towards that attitude.
class QuadrotorController {
 public:
  struct Config {
    double gravity = 9.81;               ///< [m/s^2]
    double cruise_speed = 1.0;           ///< Speed far from the waypoint [m/s].
    double distance_gain = 1.0;          ///< How fast the speed saturates with distance [1/m].
    double velocity_gain = 1.0;          ///< Acceleration per m/s of velocity error [1/s].
    double attitude_gain = 1.0;          ///< Body rate per unit of sin(attitude error) [rad/s].
    double max_thrust_to_weight = 1.3;   ///< Thrust limit as a multiple of gravity.
    double max_yaw_rate = 0.3;           ///< Limit of the body z rate [rad/s].
  };

  explicit QuadrotorController(const Config& config) : config_(config) {}

  QuadrotorCommand compute(const RigidBodyState& state, const Waypoint& target) const;

 private:
  Config config_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MOTION_QUADROTOR_CONTROLLER_H
