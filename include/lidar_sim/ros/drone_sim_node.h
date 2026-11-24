#ifndef LIDAR_SIM_ROS_DRONE_SIM_NODE_H
#define LIDAR_SIM_ROS_DRONE_SIM_NODE_H

#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "lidar_sim/motion/drone_simulator.h"
#include "lidar_sim/ros/lidar_state_publisher.h"

namespace lidar_sim {

/// ROS front end of DroneSimulator.
///
/// Besides the LiDAR state, publishes the drone ground truth on `drone_odom_gt` and the active
/// waypoint on `marker_setpoint`.
class DroneSimNode {
 public:
  /// @throws std::runtime_error on missing or invalid parameters.
  DroneSimNode(ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  /// Blocks until ROS shuts down.
  void run();

 private:
  void publish(const ros::Time& stamp);

  double update_rate_;
  std::string drone_frame_;
  DroneSimulator simulator_;
  LidarStatePublisher publisher_;
  ros::Publisher drone_odometry_pub_;
  ros::Publisher setpoint_marker_pub_;
  visualization_msgs::Marker setpoint_marker_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_ROS_DRONE_SIM_NODE_H
