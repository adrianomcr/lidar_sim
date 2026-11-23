#ifndef LIDAR_SIM_ROS_MOTION_SIM_NODE_H
#define LIDAR_SIM_ROS_MOTION_SIM_NODE_H

#include <ros/ros.h>

#include "lidar_sim/motion/motion_simulator.h"
#include "lidar_sim/ros/lidar_state_publisher.h"

namespace lidar_sim {

/// ROS front end of MotionSimulator: moves the LiDAR along a circle and publishes its state.
class MotionSimNode {
 public:
  /// @throws std::runtime_error on missing or invalid parameters.
  MotionSimNode(ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  /// Blocks until ROS shuts down.
  void run();

 private:
  double update_rate_;
  MotionSimulator simulator_;
  LidarStatePublisher publisher_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_ROS_MOTION_SIM_NODE_H
