#include "lidar_sim/ros/motion_sim_node.h"

#include "lidar_sim/ros/params.h"

namespace lidar_sim {

namespace {

MotionSimulator::Config readMotionConfig(const ros::NodeHandle& pnh) {
  MotionSimulator::Config config;
  config.gravity = getRequiredParam<double>(pnh, "gravity");
  config.start_delay = getRequiredParam<double>(pnh, "start_delay");
  config.position_gain = getRequiredParam<double>(pnh, "tracking/position_gain");
  config.velocity_gain = getRequiredParam<double>(pnh, "tracking/velocity_gain");
  config.initial_position = getVector3Param(pnh, "initial_position");
  config.initial_orientation = getRpyDegParam(pnh, "initial_rpy_deg");
  config.lidar_mount = getRpyDegParam(pnh, "lidar_mount_rpy_deg");
  config.trajectory.center = getVector3Param(pnh, "trajectory/center");
  config.trajectory.radius = getRequiredParam<double>(pnh, "trajectory/radius");
  if (pnh.hasParam("trajectory/linear_speed")) {
    const double linear_speed = getRequiredParam<double>(pnh, "trajectory/linear_speed");
    config.trajectory.angular_speed =
        angularSpeedFromLinear(linear_speed, config.trajectory.radius);
    circleAxesFromNormal(getVector3Param(pnh, "trajectory/normal"), config.trajectory.first_axis,
                         config.trajectory.second_axis);
  } else if (pnh.hasParam("trajectory/angular_speed")) {
    // Files written before linear_speed / normal.
    const Eigen::Vector3d first_axis = getVector3Param(pnh, "trajectory/first_axis");
    const Eigen::Vector3d second_axis = getVector3Param(pnh, "trajectory/second_axis");
    const Eigen::Vector3d normal = first_axis.cross(second_axis);
    if (normal.norm() == 0.0) {
      throw std::runtime_error("trajectory/first_axis and trajectory/second_axis must not be parallel");
    }
    config.trajectory.angular_speed = getRequiredParam<double>(pnh, "trajectory/angular_speed");
    circleAxesFromNormal(normal, config.trajectory.first_axis, config.trajectory.second_axis);
  } else {
    throw std::runtime_error(
        "Missing '" + pnh.resolveName("trajectory/linear_speed") +
        "'. The circle needs linear_speed and normal "
        "(older files may use angular_speed, first_axis and second_axis).");
  }
  return config;
}

}  // namespace

MotionSimNode::MotionSimNode(ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : update_rate_(getRequiredParam<double>(pnh, "update_rate")),
      simulator_(readMotionConfig(pnh)),
      publisher_(nh, pnh) {}

void MotionSimNode::run() {
  ros::Rate rate(update_rate_);
  const ros::Time start = ros::Time::now();
  ros::Time last = start;

  while (ros::ok()) {
    ros::spinOnce();
    const ros::Time now = ros::Time::now();
    simulator_.step((now - start).toSec(), (now - last).toSec());
    last = now;

    publisher_.publish(simulator_.state().position, simulator_.imu(), now);
    rate.sleep();
  }
}

}  // namespace lidar_sim
