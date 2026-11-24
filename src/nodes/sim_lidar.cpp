#include <ros/ros.h>

#include "lidar_sim/ros/lidar_sim_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_lidar");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    lidar_sim::LidarSimNode node(nh, pnh);
    node.run();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
