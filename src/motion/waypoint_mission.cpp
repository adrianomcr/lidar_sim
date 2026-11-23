#include "lidar_sim/motion/waypoint_mission.h"

#include <stdexcept>

namespace lidar_sim {

WaypointMission::WaypointMission(std::vector<Waypoint> waypoints)
    : waypoints_(std::move(waypoints)) {
  if (waypoints_.empty()) {
    throw std::invalid_argument("A mission needs at least one waypoint");
  }
  for (std::size_t i = 1; i < waypoints_.size(); ++i) {
    if (waypoints_[i].time <= waypoints_[i - 1].time) {
      throw std::invalid_argument("Waypoint times must be strictly increasing");
    }
  }
}

const Waypoint& WaypointMission::activeWaypoint(double t) const {
  const Waypoint* active = &waypoints_.front();
  for (const Waypoint& waypoint : waypoints_) {
    if (t > waypoint.time) {
      active = &waypoint;
    }
  }
  return *active;
}

}  // namespace lidar_sim
