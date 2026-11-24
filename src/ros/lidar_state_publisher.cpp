#include "lidar_sim/ros/lidar_state_publisher.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "lidar_sim/ros/conversions.h"
#include "lidar_sim/ros/params.h"

namespace lidar_sim {

LidarStatePublisher::LidarStatePublisher(ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : world_frame_(getRequiredParam<std::string>(pnh, "world_frame")),
      lidar_frame_(getRequiredParam<std::string>(pnh, "lidar_frame")),
      imu_pub_(nh.advertise<sensor_msgs::Imu>("imu/data", 10)),
      odometry_pub_(nh.advertise<nav_msgs::Odometry>("lidar_odom_gt", 10)),
      marker_pub_(nh.advertise<visualization_msgs::Marker>("marker_lidar", 1)),
      marker_(makeMarker(pnh, "lidar_marker", visualization_msgs::Marker::CYLINDER,
                         world_frame_)) {}

void LidarStatePublisher::publish(const Eigen::Vector3d& position, const ImuMeasurement& imu,
                                  const ros::Time& stamp) {
  imu_pub_.publish(toImuMsg(imu, lidar_frame_, stamp));
  odometry_pub_.publish(
      toOdometryMsg(position, imu.orientation, world_frame_, lidar_frame_, stamp));

  marker_.header.stamp = stamp;
  setMarkerPose(position, imu.orientation, marker_);
  marker_pub_.publish(marker_);
}

}  // namespace lidar_sim
