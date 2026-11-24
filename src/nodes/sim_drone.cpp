#include <ros/ros.h>

#include "lidar_sim/ros/drone_sim_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_drone");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    lidar_sim::DroneSimNode node(nh, pnh);
    node.run();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
