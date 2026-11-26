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

/// How the per-point timestamp is written into the cloud.
/// `Sim` is this simulator's original field. `Velodyne` and `Ouster` follow those ROS drivers.
enum class PointTimeFormat {
  Sim,       ///< Field `time`, float64, seconds from the start of the revolution.
  Velodyne,  ///< Field `time`, float32, seconds. Matches ros-drivers/velodyne PointcloudXYZIRT.
  Ouster,    ///< Field `t`, uint32, nanoseconds. Matches ouster_ros::Point.
};

/// @throws std::runtime_error if `name` is not `sim`, `velodyne`, or `ouster`.
PointTimeFormat pointTimeFormatFromName(const std::string& name);

/// Packs the valid returns into an unorganized cloud with fields x, y, z, intensity, the timestamp
/// selected by `time_format`, ring, azimuth, distance and return_number. The point step is 48 bytes.
void toPointCloud2(const std::vector<LidarReturn>& returns, const std::string& frame_id,
                   const ros::Time& stamp, sensor_msgs::PointCloud2& cloud,
                   PointTimeFormat time_format);

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
