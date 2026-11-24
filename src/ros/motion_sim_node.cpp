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
  config.trajectory.angular_speed = getRequiredParam<double>(pnh, "trajectory/angular_speed");
  config.trajectory.first_axis = getVector3Param(pnh, "trajectory/first_axis");
  config.trajectory.second_axis = getVector3Param(pnh, "trajectory/second_axis");
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
