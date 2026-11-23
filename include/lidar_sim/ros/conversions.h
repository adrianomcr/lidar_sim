#ifndef LIDAR_SIM_ROS_CONVERSIONS_H
#define LIDAR_SIM_ROS_CONVERSIONS_H

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "lidar_sim/lidar_scanner.h"
#include "lidar_sim/motion/rigid_body.h"

namespace lidar_sim {

/// Packs the valid returns into a Velodyne-style cloud with fields x, y, z, intensity, time, ring,
/// azimuth, distance and return_number, at fixed byte offsets with a 48-byte point step.
void toPointCloud2(const std::vector<LidarReturn>& returns, const std::string& frame_id,
                   const ros::Time& stamp, sensor_msgs::PointCloud2& cloud);

sensor_msgs::Imu toImuMsg(const ImuMeasurement& imu, const std::string& frame_id,
                          const ros::Time& stamp);

/// Ground-truth pose of `child_frame_id` in `frame_id`; the twist is left at zero.
nav_msgs::Odometry toOdometryMsg(const Eigen::Vector3d& position,
                                 const Eigen::Quaterniond& orientation,
                                 const std::string& frame_id, const std::string& child_frame_id,
                                 const ros::Time& stamp);

geometry_msgs::TransformStamped toTransformMsg(const Eigen::Vector3d& position,
                                               const Eigen::Quaterniond& orientation,
                                               const std::string& frame_id,
                                               const std::string& child_frame_id,
                                               const ros::Time& stamp);

/// Marker whose scale and color come from the parameters `<name>/scale` ([x, y, z]) and
/// `<name>/color` ([r, g, b, a]).
visualization_msgs::Marker makeMarker(const ros::NodeHandle& nh, const std::string& name,
                                      int type, const std::string& frame_id);

void setMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                   visualization_msgs::Marker& marker);

}  // namespace lidar_sim

#endif  // LIDAR_SIM_ROS_CONVERSIONS_H
