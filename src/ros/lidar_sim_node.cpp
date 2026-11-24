#include "lidar_sim/ros/lidar_sim_node.h"

#include "lidar_sim/math_utils.h"
#include "lidar_sim/ros/conversions.h"
#include "lidar_sim/ros/params.h"

namespace lidar_sim {

namespace {

LidarConfig readLidarConfig(const ros::NodeHandle& pnh) {
  LidarConfig config;
  config.scan_rate = getRequiredParam<double>(pnh, "scan_rate");
  config.num_azimuth_steps = getRequiredParam<int>(pnh, "num_azimuth_steps");
  config.num_rings = getRequiredParam<int>(pnh, "num_rings");
  config.min_elevation = degToRad(getRequiredParam<double>(pnh, "min_elevation_deg"));
  config.max_elevation = degToRad(getRequiredParam<double>(pnh, "max_elevation_deg"));
  config.min_range = getRequiredParam<double>(pnh, "min_range");
  config.max_range = getRequiredParam<double>(pnh, "max_range");
  config.intensity = static_cast<float>(getRequiredParam<double>(pnh, "intensity"));
  config.num_threads = getRequiredParam<int>(pnh, "num_threads");
  return config;
}

}  // namespace

LidarSimNode::LidarSimNode(ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : world_(loadWorldFile(getRequiredParam<std::string>(pnh, "world_file"))),
      scanner_(readLidarConfig(pnh)),
      world_frame_(getRequiredParam<std::string>(pnh, "world_frame")),
      lidar_frame_(getRequiredParam<std::string>(pnh, "lidar_frame")),
      position_(getVector3Param(pnh, "initial_position")),
      orientation_(getRpyDegParam(pnh, "initial_rpy_deg")) {
  ROS_INFO("Loaded %zu objects from %s", world_.size(),
           getRequiredParam<std::string>(pnh, "world_file").c_str());

  odometry_sub_ = nh.subscribe("lidar_odom_gt", 1, &LidarSimNode::odometryCallback, this);
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
}

void LidarSimNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  const auto& pose = msg->pose.pose;
  position_ = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  orientation_ = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z);
}

void LidarSimNode::run() {
  const double scan_period = 1.0 / scanner_.config().scan_rate;
  ros::Rate rate(scanner_.config().scan_rate);

  while (ros::ok()) {
    ros::spinOnce();
    const Eigen::Vector3d position = position_;
    const Eigen::Quaterniond orientation = orientation_;

    const ros::WallTime start = ros::WallTime::now();
    scanner_.scan(world_, position, orientation, returns_);
    const double compute_time = (ros::WallTime::now() - start).toSec();
    ROS_DEBUG("Scan computed in %.4f s", compute_time);
    if (compute_time > scan_period) {
      ROS_WARN_THROTTLE(5.0, "Scan took %.3f s, longer than the %.3f s scan period", compute_time,
                        scan_period);
    }

    // Like a real sensor, the cloud is only delivered once the revolution is complete.
    rate.sleep();

    const ros::Time stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(
        toTransformMsg(position, orientation, world_frame_, lidar_frame_, stamp));
    toPointCloud2(returns_, lidar_frame_, stamp, cloud_);
    cloud_pub_.publish(cloud_);
  }
}

}  // namespace lidar_sim
