#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>



#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/PointField.h>


#include "plane_class.h"
#include "sphere_class.h"
#include "cylinder_class.h"
#include "surface_class.h"
#include "ellipsoid_class.h"


#include <thread>


#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

#define PI 3.1415926535
#define N_A 360*4
#define N_V 16

// #define THREAD_1
// #define THREAD_2
// #define THREAD_4
#define THREAD_8







//Planes
std::vector<plane_class *> planes;
//Spheres
std::vector<sphere_class *> spheres;
//Cylinder
std::vector<cylinder_class *> cylinders;
//Surfaces
std::vector<surface_class *> surfaces;
//Ellipsoid
std::vector<ellipsoid_class *> ellipsoids;


sensor_msgs::PointCloud2 cloud_msg;



Eigen::Vector3d points_arr[N_A*N_V];
float ring_arr[N_A*N_V];
float dist_arr[N_A*N_V];
float azimuth_arr[N_A*N_V];
double time_arr[N_A*N_V];
bool flag_arr[N_A*N_V];


Eigen::Vector3d x;
Eigen::Matrix3d R;



double secs;




Eigen::Vector3d pos;
Eigen::VectorXd quat(4);
Eigen::Vector3d vel;
Eigen::Vector3d omega;
// callback for lidar ground truth odometry
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg){
  pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  quat << msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
  vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  omega << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}




void computation_task(int first, int last, float *r, float *d, float *a, double *t, Eigen::Vector3d *pts, bool *f)
{

    float g[100];
    int count;
    for (int j = first; j<last; j++){
      double phi = j*(PI/4.0)/(N_V*1.0) - PI/8.0;

      // std::cout << "" << std::endl;
      for (int i = 0; i<N_A; i++){
        double theta = i*2.0*PI/(N_A*1.0);
        theta = 2*PI-theta;
        // std::cout << theta << std::endl;

        count = 0;

        Eigen::Vector3d v0, v;
        v0 << cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi);
        v = R*v0;


        //Planes
        for(int k=0; k<planes.size(); k++){
          g[count] = planes[k]->compute_distance(x,v);
          count++;
        }

        //Spheres
        for(int k=0; k<spheres.size(); k++){
          g[count] = spheres[k]->compute_distance(x,v);
          count++;
        }

        //Cylinders
        for(int k=0; k<cylinders.size(); k++){
          g[count] = cylinders[k]->compute_distance(x,v);
          count++;
        }

        //Surfaces
        for(int k=0; k<surfaces.size(); k++){
          g[count] = surfaces[k]->compute_distance(x,v);
          count++;
        }

        //Ellipsoids
        for(int k=0; k<ellipsoids.size(); k++){
          g[count] = ellipsoids[k]->compute_distance(x,v);
          count++;
        }

        //Minimum distance
        float gamma_final;
        gamma_final = 1e6;
        for(int k=0; k<count; k++){
          if(g[k]<gamma_final){
            gamma_final = g[k];
          }
        }

        if(gamma_final>1.0 && gamma_final<100){
          // p = x+v*gamma_final; //world
          Eigen::Vector3d p;
          p = v0*gamma_final; //lidar

          double time;
          time = 0.1-theta/(2*PI)*0.1 + 0.05*0;
          if(time>0.1){time = time-0.1;}
          // time = 0.0;

          float azi;
          // azi = atan2(p(1),p(0));
          // if(azi<PI){azi = azi + 2*PI;}
          azi = atan2(-p(1),p(0));// + j*1.0/N_A/N_V*2*PI ;
          if(azi<0){azi = azi+2*PI;}
          // azi = 2*PI-azi;
	  
          // pts[j*N_A + i] = p;
          // r[j*N_A + i] = j;
          // d[j*N_A + i] = p.norm();
          // a[j*N_A + i] = azi;
          // t[j*N_A + i] = time;
          // f[j*N_A + i] = true;
          //
          pts[i*N_V + j] = p;
          r[i*N_V + j] = j;
          d[i*N_V + j] = p.norm();
          a[i*N_V + j] = azi;
          t[i*N_V + j] = time;
          f[i*N_V + j] = true;
        }
        else{
          // f[j*N_A + i] = false;
          //
          f[i*N_V + j] = false;
        }
        
      } //end for
    } //end for

} //end computation_task






int load_world_file(const std::string& filename)
{


  // std::ifstream file("/home/NEA.com/adriano.rezende/simulation_ws/src/lidar_sim/config/test.json");
  std::ifstream file(filename);
  
  if (!file.is_open()) {
      std::cerr << "\33[91mFailed to open the file.\33[0m\n";
      return 1;
  }
  json jsonData;
  file >> jsonData;

  Eigen::Vector3d axis, center, size;
  float radius, lb, ub;

  // Iterate over the fields of the JSON object
  for (auto& element : jsonData.items()) {
      // Access the key and value of each field
      const std::string& key = element.key();
      const json& value = element.value();

      const std::string& description = value["description"];
      const std::string& type = value["type"];

    if(type=="plane"){
      axis << value["axis"][0], value["axis"][1], value["axis"][2];
      center << value["center"][0], value["center"][1], value["center"][2];
      std::cout << "Adding plane: " << description << std::endl;
      planes.push_back( new plane_class(axis(0),axis(1),axis(2), center(0),center(1),center(2)));
    }
    else if(type=="sphere"){
      center << value["center"][0], value["center"][1], value["center"][2];
      radius = value["radius"];
      std::cout << "Adding sphere: " << description << std::endl;
      spheres.push_back( new sphere_class(center(0),center(1),center(2), radius));
    }
    else if(type=="cylinder"){
      axis << value["axis"][0], value["axis"][1], value["axis"][2];
      center << value["center"][0], value["center"][1], value["center"][2];
      radius = value["radius"];
      std::cout << "Adding cylinder: " << description << std::endl;
      if(value["bounds"].empty()){
        cylinders.push_back( new cylinder_class(axis(0),axis(1),axis(2), center(0),center(1),center(2), radius));
      }
      else{
        lb = value["bounds"][0];
        ub = value["bounds"][1];
        cylinders.push_back( new cylinder_class(axis(0),axis(1),axis(2), center(0),center(1),center(2), radius, lb, ub));
      }
    }
    else if(type=="ellipsoid"){
      size << value["size"][0], value["size"][1], value["size"][2];
      center << value["center"][0], value["center"][1], value["center"][2];
      std::cout << "Adding ellipsoid: " << description << std::endl;
      ellipsoids.push_back( new ellipsoid_class(size(0),size(1),size(2), center(0),center(1),center(2)));
    }
    else{
      std::cout << "\33[91mUnrecognized type: " << type << "\33[0m" << std::endl;
    }

    // std::cout << std::endl;

  }

  return 0;

}







int main(int argc, char **argv) {

//alt_ref = atof(argv[1]);

  //Initialize the node
  ros::init(argc, argv, "lidar_sim");
  ros::NodeHandle nh;

  std::string filename;

  if (argc > 1) {
    filename = argv[1];
  }

//   //Rread the curves parameters in the config files
//   ros::NodeHandle nh2("lidar_sim");
//   read_parameters(nh2);

  ros::Subscriber ekf_sub = nh.subscribe<nav_msgs::Odometry>("/lidar_odom_gt", 1, callback_odom);
  ros::Publisher pub_points = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);


  // Ground truth transform
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  std_msgs::Bool bool_msg;
  bool_msg.data = false;


  pos << 0,0,2;
  quat << 1,0,0,0;
  vel << 0,0,0;
  omega << 0,0,0;


  // if (load_world_file("/home/NEA.com/adriano.rezende/simulation_ws/src/lidar_sim/config/test.json") != 0){
  if (load_world_file(filename) != 0){
    std::cerr << "\33[91mlidar_sim failed to read world file: " << filename << "\33[0m\n";
    return 1;
  }


  x << 0.0, 0.0, 2.0;
  R << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0;

  int count = 0;

  double freq = 10;
  //Define the frequency
  ros::Rate loop_rate(freq);

  double A,B,C;
  Eigen::Vector3d Z;


  while (ros::ok()){

    secs = ros::Time::now().toSec();

    //Wait and check the callbacks
    ros::spinOnce();
    
    Eigen::Quaterniond Q;
    Q.w() = quat(0);
    Q.x() = quat(1);
    Q.y() = quat(2);
    Q.z() = quat(3);
    R = Q.normalized().toRotationMatrix();
    x = pos;

#ifdef THREAD_1
    std::thread th0(computation_task, 0,16, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    th0.join();
#endif

#ifdef THREAD_2
    std::thread th0(computation_task, 0,8, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th1(computation_task, 8,16, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    th0.join();
    th1.join();
#endif

#ifdef THREAD_4
    std::thread th0(computation_task, 0,4, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th1(computation_task, 4,8, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th2(computation_task, 8,12, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th3(computation_task, 12,16, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    th0.join();
    th1.join();
    th2.join();
    th3.join();
#endif

#ifdef THREAD_8
    std::thread th0(computation_task, 0,2, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th1(computation_task, 2,4, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th2(computation_task, 4,6, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th3(computation_task, 6,8, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th4(computation_task, 8,10, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th5(computation_task, 10,12, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th6(computation_task, 12,14, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    std::thread th7(computation_task, 14,16, ring_arr, dist_arr, azimuth_arr, time_arr, points_arr, flag_arr);
    th0.join();
    th1.join();
    th2.join();
    th3.join();
    th4.join();
    th5.join();
    th6.join();
    th7.join();
#endif



    const uint32_t POINT_STEP = 48;
    cloud_msg.header.frame_id = "lidar";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.fields.resize(9);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32; // 7
    cloud_msg.fields[0].count = 1;
    //
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32; // 7
    cloud_msg.fields[1].count = 1;
    //
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32; // 7
    cloud_msg.fields[2].count = 1;
    //
    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 16;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32; // 7
    cloud_msg.fields[3].count = 1;
    //
    cloud_msg.fields[4].name = "time";
    cloud_msg.fields[4].offset = 32;
    cloud_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT64; // 8
    cloud_msg.fields[4].count = 1;
    //
    cloud_msg.fields[5].name = "ring";
    cloud_msg.fields[5].offset = 28;
    cloud_msg.fields[5].datatype = sensor_msgs::PointField::UINT16; // 4
    cloud_msg.fields[5].count = 1;
    //
    cloud_msg.fields[6].name = "azimuth";
    cloud_msg.fields[6].offset = 20;
    cloud_msg.fields[6].datatype = sensor_msgs::PointField::FLOAT32; // 7
    cloud_msg.fields[6].count = 1;
    //
    cloud_msg.fields[7].name = "distance";
    cloud_msg.fields[7].offset = 24;
    cloud_msg.fields[7].datatype = sensor_msgs::PointField::FLOAT32; // 7
    cloud_msg.fields[7].count = 1;
    //
    cloud_msg.fields[8].name = "return_number";
    cloud_msg.fields[8].offset = 40;
    cloud_msg.fields[8].datatype = sensor_msgs::PointField::UINT8; // 2
    cloud_msg.fields[8].count = 1;
    //
    size_t n_detected_points = 0;
    for(int i = 0; i<N_A*N_V; i++){
      if(flag_arr[i]==true){
        n_detected_points++;
      }
    }
    // cloud_msg.data.resize(std::max((size_t)1, cloud_msg1.points.size()) * POINT_STEP, 0x00);
    cloud_msg.data.resize(std::max((size_t)1, n_detected_points) * POINT_STEP, 0x00);
    // cloud_msg.data.resize(n_detected_points * POINT_STEP, 0x00);
    cloud_msg.point_step = POINT_STEP;
    cloud_msg.row_step = cloud_msg.data.size();
    cloud_msg.height = 1;
    cloud_msg.width = cloud_msg.row_step / POINT_STEP;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;
    uint8_t *ptr = cloud_msg.data.data();

    for (size_t i = 0; i < N_A*N_V; i++)
    {
      // std::cout << flag_arr[i] << std::endl;
      if(flag_arr[i]==true){
        *(reinterpret_cast<float*>(ptr +  0)) = points_arr[i](0);
        *(reinterpret_cast<float*>(ptr +  4)) = points_arr[i](1);
        *(reinterpret_cast<float*>(ptr +  8)) = points_arr[i](2);
        *(reinterpret_cast<uint16_t*>(ptr +  28)) = ring_arr[i];
        // std::cout << ring_arr[i] << std::endl;
        //
        *(reinterpret_cast<double*>(ptr +  32)) = time_arr[i];
        *(reinterpret_cast<float*>(ptr +  20)) = azimuth_arr[i];
        *(reinterpret_cast<float*>(ptr +  24)) = dist_arr[i];
        //
        *(reinterpret_cast<float*>(ptr +  16)) = 60;//azimuth_arr[i];
        *(reinterpret_cast<uint8_t*>(ptr +  40)) = 1;
        
        ptr += POINT_STEP;
      }
      
    }


    std::cout << "\33[94m" << ros::Time::now().toSec()-secs << "\t" << 0.1/(ros::Time::now().toSec()-secs) << "\33[0m" << std::endl;
    if(ros::Time::now().toSec()-secs > 0.1){
        std::cout << "\33[93m[WARNING] LiDAR simulator is taking too long to compute pointcloud: " <<  ros::Time::now().toSec()-secs << " seconds\33[0m" << std::endl;
    }


    loop_rate.sleep();



    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "lidar";
    transformStamped.transform.translation.x = x(0);
    transformStamped.transform.translation.y = x(1);
    transformStamped.transform.translation.z = x(2);
    transformStamped.transform.rotation.w = quat(0);
    transformStamped.transform.rotation.x = quat(1);
    transformStamped.transform.rotation.y = quat(2);
    transformStamped.transform.rotation.z = quat(3);
    br.sendTransform(transformStamped);

    //Publish pointcloud after 0.1 seconds
    cloud_msg.header.stamp = ros::Time::now();
    pub_points.publish(cloud_msg);



  }

}






