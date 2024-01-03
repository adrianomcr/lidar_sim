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

// #include <sensor_msgs/convertPointCloud2ToPointCloud.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/PointField.h>


// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

#define PI 3.1415926535

Eigen::Vector3d x;
Eigen::Matrix3d R;


//Plane floor
Eigen::Vector3d n1;
double d1;
double gamma1;

Eigen::Vector3d n2;
double d2;
double gamma2;

Eigen::Vector3d n3;
double d3;
double gamma3;

Eigen::Vector3d c4;
double r4;
double gamma4, gamma4_1, gamma4_2;


double gamma_all;

//Direction of beam
Eigen::Vector3d v;


Eigen::Vector3d p;


double secs;


std::vector<float> ring;


int main(int argc, char **argv) {

//alt_ref = atof(argv[1]);

  //Initialize the node
  ros::init(argc, argv, "lidar_sim");
  ros::NodeHandle nh;

//   //Rread the curves parameters in the config files
//   ros::NodeHandle nh2("lidar_sim");
//   read_parameters(nh2);

// ros::Subscriber ekf_sub = nh.subscribe<nav_msgs::Odometry>("/ekf_odom", 1, GetEKf);


  ros::Publisher pub_test = nh.advertise<std_msgs::Bool>("/test", 1);
  ros::Publisher pub_points = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
  ros::Publisher pub_points1 = nh.advertise<sensor_msgs::PointCloud>("/velodyne_points_1", 1);

  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>( "/imu/data", 0 );

  ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>( "/marker_lidar", 0 );


  std_msgs::Bool bool_msg;
  bool_msg.data = false;


  sensor_msgs::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud cloud_msg1;
  // geometry_msgs::Point32 point_msg;
  

  // tf::TransformBroadcaster br;
        tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transformStamped;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.scale.x = 0.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.35;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 0.3;
  marker.color.g = 0.3;
  marker.color.b = 1.0;
  //only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";



  x << 0.0, 0.0, 2.0;
  R << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0;

  //Plane 1
  n1 << 0.0, 0.0, 1.0;
  d1 = 0;

  //Plane 2
  n2 << -1.0, 0.0, 0.0;
  d2 = -8;

  //Plane 3
  n3 << 0.0, 1.0, 0.0;
  d3 = -20;

  //Sphere
  r4 = 2;
  c4 << -10, 15, 4;


//   n2 = [-1;0;0]; n2 = n2/norm(n2);
// d2 = -8;
// n3 = [0;1;0]; n3 = n3/norm(n3);
// d3 = -20;


  double ang = 0;

  int count = 0;

  double freq = 10;
  //Define the frequency
  ros::Rate loop_rate(freq);


  double A,B,C;

  double gammas[10];
  for (int k = 0; k<10; k++){
    gammas[k] = 1e6;
  }

    while (ros::ok()){

      // x(0) += 0.1;

      secs = ros::Time::now().toSec();

      //Wait and check the callbacks
      ros::spinOnce();

      ring.clear();

      // std::cout << x.transpose() << std::endl;


      count++;
      double t = 0.1*count;
      Eigen::Vector3d vel;
      vel << cos(t), sin(t), sin(t/2.0)/10;
      x = x+0.5*vel*0.1;

      ang = ang+0.25*0.1;
      ang = 0;
      R << 1.0, 0.0, 0.0,
           0.0, cos(ang), -sin(ang),
           0.0, sin(ang), cos(ang);

      cloud_msg1.points.clear();


      //theta = linspace(0,2*pi,360)
      //      for phi = linspace(-pi/12,pi/12,16)
      for (int i = 0; i<360; i++){
        double theta = i*2.0*PI/360.0;
        for (int j = 0; j<16; j++){
          double phi = j*(PI/4.0)/16.0 - PI/8.0;

          // std::cout << i << "\t" << j << std::endl;


          Eigen::Vector3d v0;
          v0 << cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi);
          v = R*v0;


          //Plane
          gamma1 = (d1- x.dot(n1))/v.dot(n1);
          // std::cout << gamma1 << std::endl;
          if (gamma1<=0){
            gamma1 = 1e6;
          }
          gammas[0] = gamma1;

          //Plane
          gamma2 = (d2- x.dot(n2))/v.dot(n2);
          // std::cout << gamma2 << std::endl;
          if (gamma2<=0){
            gamma2 = 1e6;
          }
          gammas[1] = gamma2;

          //Plane
          gamma3 = (d3- x.dot(n3))/v.dot(n3);
          // std::cout << gamma3 << std::endl;
          if (gamma3<=0){
            gamma3 = 1e6;
          }
          gammas[2] = gamma3;

          //Sphere
          B = 2*v.dot(x-c4);
          C = (x-c4).dot(x-c4)-r4*r4;
          if(B*B-4*C >= 0){
              gamma4_1 = (-B+sqrt(B*B-4*C))/2.0; if (gamma4_1<=0) {gamma4_1 = 1e6;}
              gamma4_2 = (-B-sqrt(B*B-4*C))/2.0; if (gamma4_2<=0) {gamma4_2 = 1e6;}
              gamma4 = gamma4_1;
              if(gamma4_2 < gamma4_1){
                gamma4 = gamma4_2;
              }
          }
          else{
              gamma4 = 1e6;
          }
          gammas[3] = gamma4;

          //Minimum distance
          gamma_all = 1e6;
          for(int k=0; k<10; k++){
            if(gammas[k]<gamma_all){
              gamma_all = gammas[k];
            }
          }


          

          if(gamma_all>1.0 && gamma_all<100){
            // p = x+v*gamma_all; //world
            p = v0*gamma_all; //lidar

            geometry_msgs::Point32 point_msg;
            point_msg.x = p(0);
            point_msg.y = p(1);
            point_msg.z = p(2);

            // std::cout << point_msg << std::endl << std::endl;

            cloud_msg1.points.push_back(point_msg);

            ring.push_back(j);
            //line = [line, p, x];
            //pts = [pts, p];
          }

        }
      }



      cloud_msg1.header.stamp = ros::Time::now();
      // cloud_msg1.header.frame_id = "world";//"lidar";
      cloud_msg1.header.frame_id = "lidar";
      // cloud_msg1.height: 1
      // cloud_msg1.width: 5
      // cloud_msg1.is_bigendian = false;

      // cloud_msg1.point_step = 22;
      // cloud_msg1.row_step = 6000;
      // cloud_msg1.data = 6000
      // cloud_msg1.is_dense = true;
      // pub_points1.publish(cloud_msg1);






//####################################################################################################################################################################################################################################################
      // cloud_msg.header.stamp = ros::Time::now();
      // cloud_msg.header.frame_id = "lidar";
      // cloud_msg.height = 1;
      // int LEN = 10;
      // cloud_msg.width = LEN; //length of the pointcloud
      // // cloud_msg.fields = ;
      // cloud_msg.is_bigendian = false;
      // cloud_msg.point_step = 48;
      // cloud_msg.row_step = 48*LEN;
      // // cloud_msg.data = ;
      // cloud_msg.is_dense = true;
      // //
      // pub_points.publish(cloud_msg);

      // std::cout << "A" << std::endl;
      // g_scan_new = false;
      const uint32_t POINT_STEP = 48;
      // sensor_msgs::PointCloud2 cloud_msg;
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

      //
      //
      cloud_msg.data.resize(std::max((size_t)1, cloud_msg1.points.size()) * POINT_STEP, 0x00);
      cloud_msg.point_step = POINT_STEP;
      cloud_msg.row_step = cloud_msg.data.size();
      cloud_msg.height = 1;
      cloud_msg.width = cloud_msg.row_step / POINT_STEP;
      cloud_msg.is_bigendian = false;
      cloud_msg.is_dense = true;
      uint8_t *ptr = cloud_msg.data.data();

      for (size_t i = 0; i < cloud_msg1.points.size(); i++)
      {
        *(reinterpret_cast<float*>(ptr +  0)) = cloud_msg1.points[i].x;
        *(reinterpret_cast<float*>(ptr +  4)) = cloud_msg1.points[i].y;
        *(reinterpret_cast<float*>(ptr +  8)) = cloud_msg1.points[i].z;
        //*(reinterpret_cast<float*>(ptr + 16)) = 0;
        //*(reinterpret_cast<uint16_t*>(ptr + 20)) = cloud.points[i].r;

        *(reinterpret_cast<uint16_t*>(ptr +  28)) = ring[i];
        ptr += POINT_STEP;
      }


      uint8_t vBuffer[4];
      float pos_xyz[3];

      vBuffer[0] = 21;
      vBuffer[1] = 193;
      vBuffer[2] = 150;
      vBuffer[3] = 65;
    
    
      pos_xyz[0] = *(float *)&vBuffer;

      // 238, 1, 228, 191, - y
      vBuffer[0] = 238;
      vBuffer[1] = 1;
      vBuffer[2] = 228;
      vBuffer[3] = 191;
      pos_xyz[1] = *(float *)&vBuffer;

      // 150, 240, 161, 64, - z
      vBuffer[0] = 150;
      vBuffer[1] = 240;
      vBuffer[2] = 161;
      vBuffer[3] = 64;
      pos_xyz[2] = *(float *)&vBuffer;

      //251, 4, 193, 61, - azimuth
      vBuffer[0] = 251;
      vBuffer[1] = 4;
      vBuffer[2] = 193;
      vBuffer[3] = 62;
      float azimuth = *(float *)&vBuffer;

      float azimuth_rec = atan2(pos_xyz[0],pos_xyz[1])*3.1416/180;///3.1416;

      std::cout << "pos_xyz: " << pos_xyz[0] << " " << pos_xyz[1] << " " << pos_xyz[2] << " " << std::endl;
      std::cout << "azimuth: " << azimuth << " " << std::endl;
      std::cout << "azimuth_reconstructed: " << azimuth_rec << " " << std::endl;


      std::cout << std::endl;
      pub_points.publish(cloud_msg);

//####################################################################################################################################################################################################################################################

// header: 
//   seq: 33
//   stamp: 
//     secs: 1670513972
//     nsecs: 621129990
//   frame_id: "lidar"
// height: 1
// width: 27474
// fields: "<array type: sensor_msgs/PointField, length: 6>"
// is_bigendian: False
// point_step: 22
// row_step: 604428
// data: "<array type: uint8, length: 604428>"
// is_dense: True


      
      

      

      // sensor_msgs::PointCloud2 cloud_msg12;

      // sensor_msgs::PointCloudConstPtr test;
      // *test = &cloud_msg1;
      
      // sensor_msgs::convertPointCloud2ToPointCloud (cloud_msg1, cloud_msg12);
      // sensor_msgs::convertPointCloudToPointCloud2(test, cloud_msg12);

      // tf::Transform transform;
      // // transform.setOrigin( tf::Vector3(x(0), x(1), x(2)) );
      // // tf::Quaternion q;
      // // q.setRPY(ang, 0,0);
      // // transform.setRotation(q);
      // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lidar"));
    


      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "lidar";
      imu_msg.orientation.w = 1.0;
      imu_msg.linear_acceleration.z = 9.81;
      pub_imu.publish(imu_msg);


      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "lidar";
      transformStamped.transform.translation.x = x(0);
      transformStamped.transform.translation.y = x(1);
      transformStamped.transform.translation.z = x(2);
      tf2::Quaternion q;
      q.setRPY(ang, 0, 0);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      br.sendTransform(transformStamped);


      marker.pose.position.x = x(0);
      marker.pose.position.y = x(1);
      marker.pose.position.z = x(2);
      marker.pose.orientation.x = sin(ang/2.0);
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = cos(ang/2.0);
      pub_marker.publish( marker );



      pub_test.publish(bool_msg);


      std::cout << ros::Time::now().toSec()-secs << std::endl;

      loop_rate.sleep();


    }

}

