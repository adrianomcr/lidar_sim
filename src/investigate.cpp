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

#define POINT_STEP 48

int counter = 0;
void GetPointcloud (const sensor_msgs::PointCloud2ConstPtr& msg){
//  std::cout << msg-> << std::endl;

  counter++;

  int n = msg->width;

  int choice;

  float x,y,z,intense,azi,dist;
  uint16_t r;
  uint8_t ret;
  double time;
  // float time;

  uint8_t bts[POINT_STEP];
  uint8_t arr_1[1];
  uint8_t arr_2[2];
  uint8_t arr_4[4];
  uint8_t arr_8[8];


  choice = round(0.7*n);


  std::cout << "x" << " \t" << "y" << " \t" << "z" << " \t" << "i" << " \t" << "\33[91mt" << " \t" << "\33[92mr" << " \t" << "\33[93ma" << " \t" << "\33[0md" << " \t" << "ret" << " \t" << std::endl;


  for (int per = 0; per <100; per=per+7){
    choice = round((per/100.0)*n);
  // for (int per = 0; per <n; per=per+1){
    // choice = per;

    for (int k = 0; k<POINT_STEP; k++){
      bts[k] = msg->data[choice*POINT_STEP+k];
    }




    // KAARTA MODIFIED PACKAGE
    for (int k = 0; k<4; k++){
      arr_4[k] = bts[k];
    }
    x = *(float *)&arr_4;
    // std::cout << "x: " << x << std::endl;

    for (int k = 0; k<4; k++){
      arr_4[k] = bts[k+4];
    }
    y = *(float *)&arr_4;
    // std::cout << "y: " << y << std::endl;

    for (int k = 0; k<4; k++){
      arr_4[k] = bts[k+8];
    }
    z = *(float *)&arr_4;
    // std::cout << "z: " << z << std::endl;

    for (int k = 0; k<4; k++){
      arr_4[k] = bts[k+16];
    }
    intense = *(float *)&arr_4;
    // std::cout << "i: " << intense << std::endl;

    for (int k = 0; k<8; k++){
      arr_8[k] = bts[k+32];
    }
    time = *(double *)&arr_8;
    // std::cout << "t: " << time << "\t\t" << time*10*2*PI << std::endl;


    for (int k = 0; k<2; k++){
      arr_2[k] = bts[k+28];
    }
    r = *(uint16_t *)&arr_2;
    // std::cout << "r: " << r << std::endl;


    for (int k = 0; k<4; k++){
      arr_4[k] = bts[k+20];
    }
    azi = *(float *)&arr_4;
    float azi2 = atan2(-y,x);
    if(azi2<0){azi2 = azi2+2*3.1415926535;}
    // std::cout << "a: " << azi <<  "\t\t" << azi2 << std::endl;


    for (int k = 0; k<4; k++){
      arr_4[k] = bts[k+24];
    }
    dist = *(float *)&arr_4;
    // std::cout << "d: " << dist <<  "\t\t" << sqrt(x*x+y*y+z*z) << std::endl;



    for (int k = 0; k<1; k++){
      arr_1[k] = bts[k+40];
    }
    ret = *(uint8_t *)&arr_1;
    // std::cout << "return_mumber: " << ret << std::endl;


    std::cout << x << "  " << y << "  " << z << "  " << intense << "  \33[91m" << time << "  \33[92m" << r << "  \33[93m" << azi << "  \33[0m" << dist << "  " << ret << "  " << std::endl;

  }
  std::cout << std::endl;



  // ORIGINAL PACKAGE
  // for (int k = 0; k<4; k++){
  //   arr_4[k] = bts[k];
  // }
  // x = *(float *)&arr_4;
  // std::cout << "x: " << x << std::endl;

  // for (int k = 0; k<4; k++){
  //   arr_4[k] = bts[k+4];
  // }
  // y = *(float *)&arr_4;
  // std::cout << "y: " << y << std::endl;

  // for (int k = 0; k<4; k++){
  //   arr_4[k] = bts[k+8];
  // }
  // z = *(float *)&arr_4;
  // std::cout << "z: " << z << std::endl;

  // for (int k = 0; k<4; k++){
  //   arr_4[k] = bts[k+12];
  // }
  // intense = *(float *)&arr_4;
  // std::cout << "i: " << intense << std::endl;

  // for (int k = 0; k<2; k++){
  //   arr_2[k] = bts[k+16];
  // }
  // r = *(uint16_t *)&arr_2;
  // std::cout << "r: " << r << std::endl;

  // for (int k = 0; k<4; k++){
  //   arr_4[k] = bts[k+18];
  // }
  // time = *(float *)&arr_4;
  // std::cout << "t: " << time << std::endl;







  // uint8_t *ptr = cloud_msg.data.data();

  // for (size_t i = 0; i < cloud_msg1.points.size(); i++)
  // {
  //   *(reinterpret_cast<float*>(ptr +  0)) = cloud_msg1.points[i].x;
  //   *(reinterpret_cast<float*>(ptr +  4)) = cloud_msg1.points[i].y;
  //   *(reinterpret_cast<float*>(ptr +  8)) = cloud_msg1.points[i].z;
  //   //*(reinterpret_cast<float*>(ptr + 16)) = 0;
  //   //*(reinterpret_cast<uint16_t*>(ptr + 20)) = cloud.points[i].r;

  //   *(reinterpret_cast<uint16_t*>(ptr +  28)) = ring[i];
  //   ptr += POINT_STEP;
  // }









  
  // std::cout << "a: " << counter << std::endl;
  // std::cout << "data: " << msg->data[0] << std::endl;


  std::cout << std::endl;
}









int main(int argc, char **argv) {

  //Initialize the node
  ros::init(argc, argv, "lidar_sim");
  ros::NodeHandle nh;

  ros::Subscriber ekf_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, GetPointcloud);



  double freq = 100;
  ros::Rate loop_rate(freq);


  while (ros::ok()){

    ros::spinOnce();

    loop_rate.sleep();

  }

}

