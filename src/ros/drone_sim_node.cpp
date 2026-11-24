#include "lidar_sim/ros/drone_sim_node.h"

#include <vector>

#include <nav_msgs/Odometry.h>

#include "lidar_sim/math_utils.h"
#include "lidar_sim/ros/conversions.h"
#include "lidar_sim/ros/params.h"

namespace lidar_sim {

namespace {

double toDouble(XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  throw std::runtime_error("expected a number");
}

/// Reads a list of {time, position: [x, y, z], yaw_deg} entries.
std::vector<Waypoint> readWaypoints(const ros::NodeHandle& pnh, const std::string& name) {
  const std::string full_name = pnh.resolveName(name);
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam(name, list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("Missing or invalid parameter '" + full_name + "'");
  }

  std::vector<Waypoint> waypoints;
  for (int i = 0; i < list.size(); ++i) {
    XmlRpc::XmlRpcValue& entry = list[i];
    const std::string entry_name = full_name + "[" + std::to_string(i) + "]";
    try {
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || !entry.hasMember("time") ||
          !entry.hasMember("position") || !entry.hasMember("yaw_deg")) {
        throw std::runtime_error("expected {time, position, yaw_deg}");
      }
      XmlRpc::XmlRpcValue& position = entry["position"];
      if (position.getType() != XmlRpc::XmlRpcValue::TypeArray || position.size() != 3) {
        throw std::runtime_error("'position' must have 3 elements");
      }
      Waypoint waypoint;
      waypoint.time = toDouble(entry["time"]);
      waypoint.position =
          Eigen::Vector3d(toDouble(position[0]), toDouble(position[1]), toDouble(position[2]));
      waypoint.yaw = degToRad(toDouble(entry["yaw_deg"]));
      waypoints.push_back(waypoint);
    } catch (const std::exception& e) {
      throw std::runtime_error("Invalid waypoint '" + entry_name + "': " + e.what());
    }
  }
  return waypoints;
}

DroneSimulator::Config readDroneConfig(const ros::NodeHandle& pnh) {
  DroneSimulator::Config config;
  config.gravity = getRequiredParam<double>(pnh, "gravity");
  config.drag_coefficient = getRequiredParam<double>(pnh, "drag_coefficient");
  config.hover_duration = getRequiredParam<double>(pnh, "hover_duration");
  config.min_attitude_control_altitude =
      getRequiredParam<double>(pnh, "min_attitude_control_altitude");
  config.initial_position = getVector3Param(pnh, "initial_position");
  config.initial_orientation = getRpyDegParam(pnh, "initial_rpy_deg");
  config.lidar_mount = getRpyDegParam(pnh, "lidar_mount_rpy_deg");

  QuadrotorController::Config& controller = config.controller;
  controller.gravity = config.gravity;
  controller.cruise_speed = getRequiredParam<double>(pnh, "controller/cruise_speed");
  controller.distance_gain = getRequiredParam<double>(pnh, "controller/distance_gain");
  controller.velocity_gain = getRequiredParam<double>(pnh, "controller/velocity_gain");
  controller.attitude_gain = getRequiredParam<double>(pnh, "controller/attitude_gain");
  controller.max_thrust_to_weight =
      getRequiredParam<double>(pnh, "controller/max_thrust_to_weight");
  controller.max_yaw_rate = getRequiredParam<double>(pnh, "controller/max_yaw_rate");

  config.waypoints = readWaypoints(pnh, "waypoints");
  return config;
}

}  // namespace

DroneSimNode::DroneSimNode(ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : update_rate_(getRequiredParam<double>(pnh, "update_rate")),
      drone_frame_(getRequiredParam<std::string>(pnh, "drone_frame")),
      simulator_(readDroneConfig(pnh)),
      publisher_(nh, pnh),
      drone_odometry_pub_(nh.advertise<nav_msgs::Odometry>("drone_odom_gt", 10)),
      setpoint_marker_pub_(nh.advertise<visualization_msgs::Marker>("marker_setpoint", 1)),
      setpoint_marker_(makeMarker(pnh, "setpoint_marker", visualization_msgs::Marker::SPHERE,
                                  publisher_.worldFrame())) {}

void DroneSimNode::run() {
  ros::Rate rate(update_rate_);
  const ros::Time start = ros::Time::now();
  ros::Time last = start;

  while (ros::ok()) {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    simulator_.step((now - start).toSec(), (now - last).toSec());
    last = now;

    publish(now);
    rate.sleep();
  }
}

void DroneSimNode::publish(const ros::Time& stamp) {
  const RigidBodyState& state = simulator_.state();
  publisher_.publish(state.position, simulator_.imu(), stamp);

  drone_odometry_pub_.publish(toOdometryMsg(state.position, state.orientation,
                                            publisher_.worldFrame(), drone_frame_, stamp));

  setpoint_marker_.header.stamp = stamp;
  setMarkerPose(simulator_.target().position, Eigen::Quaterniond::Identity(), setpoint_marker_);
  setpoint_marker_pub_.publish(setpoint_marker_);
}

}  // namespace lidar_sim
