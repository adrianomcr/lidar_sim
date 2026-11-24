// Debugging tool: prints the fields of a few points of every cloud received on `velodyne_points`,
// to check the cloud layout expected by downstream consumers.
//
// Private parameters:
//   num_samples (int, default 15): number of points printed per cloud, evenly spread.

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {

int g_num_samples = 15;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  const std::size_t num_points = static_cast<std::size_t>(msg->width) * msg->height;
  if (num_points == 0) {
    std::cout << "Empty cloud" << std::endl;
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<double> time(*msg, "time");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(*msg, "ring");
  sensor_msgs::PointCloud2ConstIterator<float> azimuth(*msg, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> distance(*msg, "distance");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> return_number(*msg, "return_number");

  std::cout << num_points << " points\n"
            << std::setw(10) << "x" << std::setw(10) << "y" << std::setw(10) << "z"
            << std::setw(10) << "intensity" << std::setw(10) << "time" << std::setw(6) << "ring"
            << std::setw(10) << "azimuth" << std::setw(10) << "distance" << std::setw(8)
            << "return" << '\n';

  const std::size_t num_samples = std::min<std::size_t>(std::max(g_num_samples, 1), num_points);
  std::size_t sample = 0;
  for (std::size_t i = 0; i < num_points && sample < num_samples; ++i) {
    if (i == sample * num_points / num_samples) {
      std::cout << std::fixed << std::setprecision(3) << std::setw(10) << *x << std::setw(10)
                << *y << std::setw(10) << *z << std::setw(10) << *intensity
                << std::setprecision(5) << std::setw(10) << *time << std::setw(6) << *ring
                << std::setprecision(3) << std::setw(10) << *azimuth << std::setw(10)
                << *distance << std::setw(8) << static_cast<int>(*return_number) << '\n';
      ++sample;
    }
    ++x, ++y, ++z, ++intensity, ++time, ++ring, ++azimuth, ++distance, ++return_number;
  }
  std::cout << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "investigate");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param("num_samples", g_num_samples, g_num_samples);

  ros::Subscriber cloud_sub = nh.subscribe("velodyne_points", 1, cloudCallback);
  ros::spin();
  return 0;
}
