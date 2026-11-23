#ifndef LIDAR_SIM_ROS_LIDAR_STATE_PUBLISHER_H
#define LIDAR_SIM_ROS_LIDAR_STATE_PUBLISHER_H

#include <string>

#include <Eigen/Core>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "lidar_sim/motion/rigid_body.h"

namespace lidar_sim {

/// Publishes the motion of the LiDAR rig, whose IMU shares the LiDAR frame:
/// IMU readings on `imu/data`, ground-truth LiDAR odometry on `lidar_odom_gt` (the pose
/// sim_lidar follows) and an RViz marker on `marker_lidar`.
///
/// Reads `world_frame`, `lidar_frame` and `lidar_marker/{scale,color}` from the private namespace.
class LidarStatePublisher {
 public:
  LidarStatePublisher(ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  /// @param position LiDAR position in the world frame.
  /// @param imu      IMU reading; its orientation is the LiDAR orientation.
  void publish(const Eigen::Vector3d& position, const ImuMeasurement& imu, const ros::Time& stamp);

  const std::string& worldFrame() const { return world_frame_; }

 private:
  std::string world_frame_;
  std::string lidar_frame_;
  ros::Publisher imu_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_ROS_LIDAR_STATE_PUBLISHER_H
