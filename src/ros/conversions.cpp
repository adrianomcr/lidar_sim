#include "lidar_sim/ros/conversions.h"

#include <cmath>
#include <cstring>
#include <cstdint>
#include <stdexcept>

#include <sensor_msgs/PointField.h>

#include "lidar_sim/ros/params.h"

namespace lidar_sim {

namespace {

// Byte layout of one point. Downstream consumers rely on these exact offsets, padding included.
constexpr std::size_t kOffsetX = 0;
constexpr std::size_t kOffsetY = 4;
constexpr std::size_t kOffsetZ = 8;
constexpr std::size_t kOffsetIntensity = 16;
constexpr std::size_t kOffsetAzimuth = 20;
constexpr std::size_t kOffsetDistance = 24;
constexpr std::size_t kOffsetRing = 28;
constexpr std::size_t kOffsetTime = 32;
constexpr std::size_t kOffsetReturnNumber = 40;
constexpr std::size_t kPointStep = 48;

// The simulated sensor reports a single (first) return per firing.
constexpr std::uint8_t kReturnNumber = 1;

sensor_msgs::PointField makeField(const std::string& name, std::size_t offset,
                                  std::uint8_t datatype) {
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = static_cast<std::uint32_t>(offset);
  field.datatype = datatype;
  field.count = 1;
  return field;
}

template <typename T>
void writeValue(std::uint8_t* point, std::size_t offset, T value) {
  std::memcpy(point + offset, &value, sizeof(T));
}

geometry_msgs::Quaternion toQuaternionMsg(const Eigen::Quaterniond& q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

geometry_msgs::Vector3 toVector3Msg(const Eigen::Vector3d& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

geometry_msgs::Point toPointMsg(const Eigen::Vector3d& v) {
  geometry_msgs::Point msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

}  // namespace

PointTimeFormat pointTimeFormatFromName(const std::string& name) {
  if (name == "sim") {
    return PointTimeFormat::Sim;
  }
  if (name == "velodyne") {
    return PointTimeFormat::Velodyne;
  }
  if (name == "ouster") {
    return PointTimeFormat::Ouster;
  }
  throw std::runtime_error("point_time must be 'sim', 'velodyne', or 'ouster'");
}

void toPointCloud2(const std::vector<LidarReturn>& returns, const std::string& frame_id,
                   const ros::Time& stamp, sensor_msgs::PointCloud2& cloud,
                   PointTimeFormat time_format) {
  using sensor_msgs::PointField;
  const char* time_name = "time";
  std::uint8_t time_type = PointField::FLOAT64;
  if (time_format == PointTimeFormat::Velodyne) {
    time_type = PointField::FLOAT32;
  } else if (time_format == PointTimeFormat::Ouster) {
    time_name = "t";
    time_type = PointField::UINT32;
  }

  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.fields = {
      makeField("x", kOffsetX, PointField::FLOAT32),
      makeField("y", kOffsetY, PointField::FLOAT32),
      makeField("z", kOffsetZ, PointField::FLOAT32),
      makeField("intensity", kOffsetIntensity, PointField::FLOAT32),
      makeField(time_name, kOffsetTime, time_type),
      makeField("ring", kOffsetRing, PointField::UINT16),
      makeField("azimuth", kOffsetAzimuth, PointField::FLOAT32),
      makeField("distance", kOffsetDistance, PointField::FLOAT32),
      makeField("return_number", kOffsetReturnNumber, PointField::UINT8),
  };

  std::size_t num_points = 0;
  for (const LidarReturn& hit : returns) {
    num_points += hit.valid ? 1 : 0;
  }

  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(num_points);
  cloud.point_step = kPointStep;
  cloud.row_step = static_cast<std::uint32_t>(num_points * kPointStep);
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.data.assign(num_points * kPointStep, 0);

  std::uint8_t* point = cloud.data.data();
  for (const LidarReturn& hit : returns) {
    if (!hit.valid) {
      continue;
    }
    writeValue(point, kOffsetX, static_cast<float>(hit.point.x()));
    writeValue(point, kOffsetY, static_cast<float>(hit.point.y()));
    writeValue(point, kOffsetZ, static_cast<float>(hit.point.z()));
    writeValue(point, kOffsetIntensity, hit.intensity);
    writeValue(point, kOffsetAzimuth, hit.azimuth);
    writeValue(point, kOffsetDistance, hit.distance);
    writeValue(point, kOffsetRing, hit.ring);
    if (time_format == PointTimeFormat::Velodyne) {
      writeValue(point, kOffsetTime, static_cast<float>(hit.time));
    } else if (time_format == PointTimeFormat::Ouster) {
      const double nanos = hit.time * 1.0e9;
      const std::uint32_t ticks =
          nanos <= 0.0 ? 0u
                       : nanos >= 4294967295.0 ? 4294967295u
                                               : static_cast<std::uint32_t>(std::llround(nanos));
      writeValue(point, kOffsetTime, ticks);
    } else {
      writeValue(point, kOffsetTime, hit.time);
    }
    writeValue(point, kOffsetReturnNumber, kReturnNumber);
    point += kPointStep;
  }
}

sensor_msgs::Imu toImuMsg(const ImuMeasurement& imu, const std::string& frame_id,
                          const ros::Time& stamp) {
  sensor_msgs::Imu msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.orientation = toQuaternionMsg(imu.orientation);
  msg.angular_velocity = toVector3Msg(imu.angular_velocity);
  msg.linear_acceleration = toVector3Msg(imu.linear_acceleration);
  return msg;
}

nav_msgs::Odometry toOdometryMsg(const Eigen::Vector3d& position,
                                 const Eigen::Quaterniond& orientation,
                                 const std::string& frame_id, const std::string& child_frame_id,
                                 const ros::Time& stamp) {
  nav_msgs::Odometry msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.child_frame_id = child_frame_id;
  msg.pose.pose.position = toPointMsg(position);
  msg.pose.pose.orientation = toQuaternionMsg(orientation);
  return msg;
}

geometry_msgs::TransformStamped toTransformMsg(const Eigen::Vector3d& position,
                                               const Eigen::Quaterniond& orientation,
                                               const std::string& frame_id,
                                               const std::string& child_frame_id,
                                               const ros::Time& stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.child_frame_id = child_frame_id;
  msg.transform.translation = toVector3Msg(position);
  msg.transform.rotation = toQuaternionMsg(orientation);
  return msg;
}

visualization_msgs::Marker makeMarker(const ros::NodeHandle& nh, const std::string& name,
                                      int type, const std::string& frame_id) {
  const Eigen::Vector3d scale = getVector3Param(nh, name + "/scale");
  const auto color = getRequiredParam<std::vector<double>>(nh, name + "/color");
  if (color.size() != 4) {
    throw std::runtime_error("Parameter '" + nh.resolveName(name + "/color") +
                             "' must be [r, g, b, a]");
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = name;
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.scale = toVector3Msg(scale);
  marker.color.r = static_cast<float>(color[0]);
  marker.color.g = static_cast<float>(color[1]);
  marker.color.b = static_cast<float>(color[2]);
  marker.color.a = static_cast<float>(color[3]);
  marker.pose.orientation.w = 1.0;
  return marker;
}

void setMarkerPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                   visualization_msgs::Marker& marker) {
  marker.pose.position = toPointMsg(position);
  marker.pose.orientation = toQuaternionMsg(orientation);
}

}  // namespace lidar_sim
