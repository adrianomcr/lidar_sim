#ifndef LIDAR_SIM_ROS_LIDAR_SIM_NODE_H
#define LIDAR_SIM_ROS_LIDAR_SIM_NODE_H

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include "lidar_sim/lidar_scanner.h"
#include "lidar_sim/ros/conversions.h"
#include "lidar_sim/world.h"

namespace lidar_sim {

/// ROS front end of the LiDAR simulator.
///
/// Follows the sensor pose received on `lidar_odom_gt`, simulates one revolution per scan
/// period against the world file, and publishes the cloud on `velodyne_points` together with
/// the ground-truth transform `world_frame` -> `tf_child_frame`. The cloud stays in
/// `lidar_frame`. Parameters are read from the private namespace.
class LidarSimNode {
 public:
  /// @throws std::runtime_error on missing parameters or an invalid world file.
  LidarSimNode(ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  /// Blocks until ROS shuts down.
  void run();

 private:
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  World world_;
  LidarScanner scanner_;
  std::string world_frame_;
  std::string lidar_frame_;
  std::string tf_child_frame_;
  PointTimeFormat point_time_ = PointTimeFormat::Sim;

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;

  ros::Subscriber odometry_sub_;
  ros::Publisher cloud_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::vector<LidarReturn> returns_;
  sensor_msgs::PointCloud2 cloud_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_ROS_LIDAR_SIM_NODE_H
