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
#define G 9.81

using namespace std;
using namespace Eigen;

Eigen::Vector3d x, v, w, a;
Eigen::Matrix3d R;
Eigen::VectorXd q(4);

Eigen::Vector3d gyro, acc;



double secs, secs_last, dt, secs_init;




Eigen::VectorXd quat_mult(VectorXd q10, VectorXd q20){
Quaterniond q1,q2, q3;
// VectorXd 
q1.w() = q10(0);
q1.x() = q10(1);
q1.y() = q10(2);
q1.z() = q10(3);
q2.w() = q20(0);
q2.x() = q20(1);
q2.y() = q20(2);
q2.z() = q20(3);

q3 = q1*q2;

VectorXd q(4);
q << q3.w(), q3.x(), q3.y(), q3.z();
return q;
}

Eigen::VectorXd quat_deriv(VectorXd q0, VectorXd w0){
//Assuming w0 on the body frame

  VectorXd w(3);
  VectorXd q(4), wq(4), dqdt(4);
	q = q0;
  w = w0;
  wq << 0.0, w(0), w(1), w(2);
  // VectorXd w(3);
  // w = w0;

  dqdt = 0.5*quat_mult(q,wq);

  return dqdt;

}



Eigen::VectorXd quat_conj(VectorXd q0){
  VectorXd q(4), q_conj(4);
  q = q0;
  q_conj << q(0), -q(1), -q(2), -q(3);

  return q_conj;
}


Eigen::VectorXd quat_rot_v(VectorXd q0, VectorXd v0){
  VectorXd q(4), vq(4), qout(4);
  VectorXd v(3), vout(3);
	q = q0;
  v = v0;
  vq << 0.0, v;

  // std::cout << "a" << std::endl;
  qout = quat_mult(quat_mult(q,vq),quat_conj(q));
  // std::cout << "b" << std::endl;
  vout << qout(1), qout(2), qout(3);
  // std::cout << "c" << std::endl;

  return vout;

}

Eigen::VectorXd rotm2quat(MatrixXd R){
  Matrix3d R2;
	R2 = R;
  Quaterniond q(R2);

  VectorXd q2(4);
  q2 << q.w(), q.x(), q.y(), q.z();

  return q2;

}

int main(int argc, char **argv) {

//alt_ref = atof(argv[1]);

  //Initialize the node
  ros::init(argc, argv, "motion_sim");
  ros::NodeHandle nh;

  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>( "/imu/data", 0 );
  ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>( "/marker_lidar", 0 );

  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>( "/lidar_odom_gt", 0 );

  nav_msgs::Odometry odom_msg;


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



  // x << 2.0, 0.0, 2.0;
  // x << -30.0, 2.0, 20.0;
  x << 0.0, 20.0, 4.0;
  v << 0.0, 0.0, 0.0;
  w << 0.0, 0.0, 0.0;
  a << 0.0, 0.0, 0.0;
  R << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0;
  q << 1.0, 0.0, 0.0, 1.0*0;
  q = q/q.norm();

// v = v*0;

//   n2 = [-1;0;0]; n2 = n2/norm(n2);
// d2 = -8;
// n3 = [0;1;0]; n3 = n3/norm(n3);
// d3 = -20;


  double freq = 50;
  //Define the frequency
  ros::Rate loop_rate(freq);


    secs = ros::Time::now().toSec();
    secs_init = secs;
    while (ros::ok()){

      //Wait and check the callbacks
      ros::spinOnce();
      
      secs_last = secs;
      secs = ros::Time::now().toSec();
      dt = secs-secs_last;


      Eigen::Vector3d pref, vref, aref;

      double t, radius, f;
      f = 0.05;
      t = secs-secs_init;
      radius = 3;
      pref << radius*cos(f*2*PI*t), radius*sin(f*2*PI*t), 2;
      vref << -(f*2*PI)*radius*sin(0.2*2*PI*t), (f*2*PI)*radius*cos(f*2*PI*t), 0;
      aref << -(f*2*PI)*(f*2*PI)*radius*cos(f*2*PI*t), -(f*2*PI)*(f*2*PI)*radius*sin(f*2*PI*t), 0;
      
      // pref << -10.0*t, -10.0*t, 2+5*t;
      pref << -12, 6*cos(0.4*t), 6*sin(0.4*t)+7;
      // vref << -10.0, 0, 1;
      vref << 0, -2.4*sin(0.4*t), 2.4*cos(0.4*t);
      aref << 0,0,0;

      vref = vref + 0.2*(pref-x);
      a = 0.4*(vref-v) + aref;

      // std::cout << "t: " << t << std::endl;
      // std::cout << "pref: " << pref.transpose() << std::endl;


      Eigen::Vector3d wref, alpharef;
      wref << cos(0.4*2*PI*t), radius*cos(0.2*2*PI*t), 0;
      // wref << 0,1,0;
      w = wref/10*20/20*0;


      // a = a*0;
      // w << 0.4, 0.21324321*0, 0.0;
      if(t<5){
        a = a*0;
        w = w*0;
      }
      if(t<10){
        a = a*0;
        // w = w*0;
      }
      // a = a*0;



      // a << -v(1), v(0), 0.0;
      // a = a/2.0;

      // a = V*V/R
      // a = 2*2/2

      // Eigen::VectorXd aaa(4);
      // aaa = quat_deriv(q,w);
      // w << 0,0,10;
      // q << 1,1,0,0;
      // // q = q/q.norm();
      // cout << "q: "<< q.transpose() << endl;
      // cout << "w: "<< w.transpose() << endl;
      // cout << "quat_conj(q): "<< quat_conj(q).transpose() << endl;
      // cout << "quat_mult(q,quat_conj(q)): "<< quat_mult(q,quat_conj(q)).transpose() << endl;
      // cout << "quat_rot_v(q,w): "<< quat_rot_v(q,w).transpose() << endl;
      // // cout << "quat_deriv(q,w): "<< quat_deriv(q,w).transpose() << endl;

      // cout << "-------------------------" << endl;



      x = x + v*dt;
      v = v + a*dt;
      q = q + quat_deriv(q,w)*dt;
      q = q/q.norm();

      

      gyro = w;
      acc = a;
      acc[2] += G;
      // std::cout << "A" << std::endl;

      acc = quat_rot_v(quat_conj(q),acc);

      // std::cout << "B" << std::endl;

      // q = rotm2quat(R);




      // count++;
      // double t = 0.1*count;
      // Eigen::Vector3d vel;
      // vel << cos(t), sin(t), sin(t/2.0)/10;
      // x = x+0.5*vel*0.1;

      // ang = ang+0.25*0.1;
      // ang = 0;
      // R << 1.0, 0.0, 0.0,
      //      0.0, cos(ang), -sin(ang),
      //      0.0, sin(ang), cos(ang);








      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "lidar";
      imu_msg.orientation.w = q(0);
      imu_msg.orientation.x = q(1);
      imu_msg.orientation.y = q(2);
      imu_msg.orientation.z = q(3);
      imu_msg.angular_velocity.x = gyro(0);
      imu_msg.angular_velocity.y = gyro(1);
      imu_msg.angular_velocity.z = gyro(2);
      imu_msg.linear_acceleration.x = acc(0);
      imu_msg.linear_acceleration.y = acc(1);
      imu_msg.linear_acceleration.z = acc(2);
      pub_imu.publish(imu_msg);


      odom_msg.header.stamp = ros::Time::now();
      odom_msg.header.frame_id = "world";
      odom_msg.pose.pose.position.x = x(0);
      odom_msg.pose.pose.position.y = x(1);
      odom_msg.pose.pose.position.z = x(2);
      odom_msg.pose.pose.orientation.w = q(0);
      odom_msg.pose.pose.orientation.x = q(1);
      odom_msg.pose.pose.orientation.y = q(2);
      odom_msg.pose.pose.orientation.z = q(3);

      pub_odom.publish(odom_msg);



      marker.header.stamp = ros::Time::now();
      marker.pose.position.x = x(0);
      marker.pose.position.y = x(1);
      marker.pose.position.z = x(2);
      marker.pose.orientation.w = q(0);
      marker.pose.orientation.x = q(1);
      marker.pose.orientation.y = q(2);
      marker.pose.orientation.z = q(3);
      pub_marker.publish( marker );




      loop_rate.sleep();


    }

}

