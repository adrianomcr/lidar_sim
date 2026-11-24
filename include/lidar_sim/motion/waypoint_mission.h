#ifndef LIDAR_SIM_MOTION_WAYPOINT_MISSION_H
#define LIDAR_SIM_MOTION_WAYPOINT_MISSION_H

#include <vector>

#include <Eigen/Core>

namespace lidar_sim {

struct Waypoint {
  double time = 0.0;  ///< Mission time after which this waypoint becomes the target [s].
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  ///< World frame [m].
  double yaw = 0.0;                                     ///< Heading to hold [rad].
};

/// Time-scheduled list of waypoints. The first waypoint is active from the start; each later one
/// replaces it once the mission time exceeds its `time`.
class WaypointMission {
 public:
  /// @throws std::invalid_argument if the list is empty or the times are not increasing.
  explicit WaypointMission(std::vector<Waypoint> waypoints);

  const Waypoint& activeWaypoint(double t) const;

 private:
  std::vector<Waypoint> waypoints_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_MOTION_WAYPOINT_MISSION_H
