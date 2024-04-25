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

#define pi 3.1415926535
#define grav 9.81

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

Eigen::VectorXd quat2axang(VectorXd q0){
  VectorXd q(4);
	q = q0;
  q = q/q.norm();
  if(q(0)<0){
    q = -q;
  }

  VectorXd axang(4);
  axang(0) = 2*acos(q(0));
  axang.block(1,0,3,1) = q.block(1,0,3,1)/q.block(1,0,3,1).norm();


  return axang;

}

int main(int argc, char **argv) {

//alt_ref = atof(argv[1]);

  //Initialize the node
  ros::init(argc, argv, "motion_sim");
  ros::NodeHandle nh;

  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>( "/imu/data", 0 );
  ros::Publisher pub_marker_lidar = nh.advertise<visualization_msgs::Marker>( "/marker_lidar", 0 );
  ros::Publisher pub_marker_setpoint = nh.advertise<visualization_msgs::Marker>( "/marker_setpoint", 0 );

  ros::Publisher pub_odom_lidar = nh.advertise<nav_msgs::Odometry>( "/lidar_odom_gt", 0 );
  ros::Publisher pub_odom_drone = nh.advertise<nav_msgs::Odometry>( "/drone_odom_gt", 0 );

  nav_msgs::Odometry odom_msg_lidar;
  nav_msgs::Odometry odom_msg_drone;


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


  visualization_msgs::Marker marker_setpoint;
  marker_setpoint.header.frame_id = "world";
  marker_setpoint.header.stamp = ros::Time();
  marker_setpoint.ns = "my_namespace";
  marker_setpoint.id = 0;
  marker_setpoint.type = visualization_msgs::Marker::SPHERE;
  marker_setpoint.action = visualization_msgs::Marker::MODIFY;
  marker_setpoint.scale.x = 0.4;
  marker_setpoint.scale.y = 0.4;
  marker_setpoint.scale.z = 0.4;
  marker_setpoint.color.a = 0.7; // Don't forget to set the alpha!
  marker_setpoint.color.r = 1.0;
  marker_setpoint.color.g = 0.0;
  marker_setpoint.color.b = 0.0;


  // x << 2.0, 0.0, 2.0;
  // x << -30.0, 2.0, 20.0;
  x << -10.0*0, 10.0*0, 2.0;
  v << 0.0, 0.0, 0.0;
  w << 0.0, 0.0, 0.0;
  a << 0.0, 0.0, 0.0;
  R << 1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0;
  q << 1.0, 0.0, 0.0, 1.0*0;
  q = q/q.norm();


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
      // t = t*4;
      radius = 3;
      pref << radius*cos(f*2*pi*t), radius*sin(f*2*pi*t), 2;
      vref << -(f*2*pi)*radius*sin(0.2*2*pi*t), (f*2*pi)*radius*cos(f*2*pi*t), 0;
      aref << -(f*2*pi)*(f*2*pi)*radius*cos(f*2*pi*t), -(f*2*pi)*(f*2*pi)*radius*sin(f*2*pi*t), 0;
      
      // pref << -10.0*t, -10.0*t, 2+5*t;
      pref << -12, 6*cos(0.4*t), 6*sin(0.4*t)+7;
      // vref << -10.0, 0, 1;
      vref << 0, -2.4*sin(0.4*t), 2.4*cos(0.4*t);
      aref << 0,0,0;

      vref = vref + 0.2*(pref-x);
      a = 0.4*(vref-v) + aref;


      Eigen::Vector3d wref, alpharef;
      wref << cos(0.4*2*pi*t), radius*cos(0.2*2*pi*t), 0;
      // wref << 0,1,0;
      w = wref/10*20/20*0;

      double t_mult = 1.5;
      //#################################
      pref << -10.0,10.0,10;
      double psiref = -pi/2;
      if(t>15*t_mult){
        pref << -15.0,10.0,10;
        psiref = 0;
      }
      if(t>25*t_mult){
        pref << -15.0,15.0,10;
      }
      if(t>35*t_mult){
        pref << -25.0,15.0,10;
        psiref = pi/2;
      }
      if(t>35*t_mult){
        pref << -35.0,15.0,10;
        psiref = 0;
      }
      if(t>45*t_mult){
        pref << -45.0,15.0,7;
        psiref = 0;
      }
      if(t>55*t_mult){
        pref << -45.0,15.0,14;
        psiref = 0;
      }
      if(t>65*t_mult){
        pref << -45.0,-15.0,14;
        psiref = 0;
      }
      double kp = 1.0, kv = 1.5, kw = 3, vr = 2.5/t_mult;
      double G = (2/pi)*atan(kp*(x-pref).norm());
      Eigen::Vector3d grad_D = (x-pref)/((x-pref).norm() + 1e-6);
      vref = -vr*G*grad_D;
      Eigen::Vector3d ad = kv*(vref-v);
      Eigen::Vector3d g_vec;
      g_vec << 0.0,0.0,-grav;
      Eigen::Vector3d ar = ad - g_vec;
      Eigen::Vector3d xRr, yRr, zRr;
      zRr = ar/ar.norm();
      xRr << cos(psiref), sin(psiref), 0;
      xRr = xRr-xRr.dot(zRr)*zRr;
      xRr = xRr/xRr.norm();
      yRr = zRr.cross(xRr);
      Eigen::Matrix3d Rr;
      Rr << xRr(0), yRr(0), zRr(0),
            xRr(1), yRr(1), zRr(1),
            xRr(2), yRr(2), zRr(2);
      Eigen::VectorXd qref(4), qe(4), axang(4);
      qref = rotm2quat(Rr);
      qe = quat_mult(quat_conj(q),qref);
      axang = quat2axang(qe);
      double beta;
      Vector3d n;
      beta = axang(0);
      n = axang.block(1,0,3,0);
       //#################################


      double tau = ar.dot(zRr);
      if(tau>1.3*grav){tau=1.3*grav;}
      w = kw*sin(beta)*n;
      double wzmax = 0.3;
      if(w(2)>wzmax){w(2) = wzmax;}
      if(w(2)<-wzmax){w(2) = -wzmax;}

      Vector3d a_body;
      a_body << 0.0, 0.0, tau;
      a = quat_rot_v(q,a_body);
      // cout << "a: " << a.transpose() << endl;
      // w << 0,0,0.4*0;





      // a = a*0;
      // w << 0.4, 0.21324321*0, 0.0;
      if(t<10){
        a = -g_vec;
        w = w*0;
      }
      if(x(2)<5){w=w*0;}
      // if(t<10){
      //   a = a*0;
      //   // w = w*0;
      // }
      // // a = a*0;



      double drag = 0.1;

      x = x + v*dt;
      v = v + (a+g_vec-drag*v)*dt;
      q = q + quat_deriv(q,w)*dt;
      q = q/q.norm();

      
      VectorXd q_aux(4), q_offset(4);
      // q_offset << 0.9829629, 0.1294095, 0.1294095, 0.0170371; //XYZ
      q_offset << 0.9829629, 0.1294095, 0.1294095, -0.0170371; //ZYX
      // q_offset << 0.999, 0.001, 0.001, 0.001;

      gyro = quat_rot_v(quat_conj(q_offset),w);
      // acc[2] += grav;
      acc = quat_rot_v(quat_conj(q),a);
      acc = quat_rot_v(quat_conj(q_offset),acc);

      //#####################################################################



      VectorXd q_vlp(4);
      q_vlp = quat_mult(q,q_offset);
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "lidar";
      imu_msg.orientation.w = q_vlp(0);
      imu_msg.orientation.x = q_vlp(1);
      imu_msg.orientation.y = q_vlp(2);
      imu_msg.orientation.z = q_vlp(3);
      imu_msg.angular_velocity.x = gyro(0);
      imu_msg.angular_velocity.y = gyro(1);
      imu_msg.angular_velocity.z = gyro(2);
      imu_msg.linear_acceleration.x = acc(0);
      imu_msg.linear_acceleration.y = acc(1);
      imu_msg.linear_acceleration.z = acc(2);
      pub_imu.publish(imu_msg);


      Vector3d x_vlp = x;
      odom_msg_lidar.header.stamp = ros::Time::now();
      odom_msg_lidar.header.frame_id = "world";
      odom_msg_lidar.child_frame_id = "lidar";
      odom_msg_lidar.pose.pose.position.x = x_vlp(0);
      odom_msg_lidar.pose.pose.position.y = x_vlp(1);
      odom_msg_lidar.pose.pose.position.z = x_vlp(2);
      odom_msg_lidar.pose.pose.orientation.w = q_vlp(0);
      odom_msg_lidar.pose.pose.orientation.x = q_vlp(1);
      odom_msg_lidar.pose.pose.orientation.y = q_vlp(2);
      odom_msg_lidar.pose.pose.orientation.z = q_vlp(3);
      pub_odom_lidar.publish(odom_msg_lidar);

      odom_msg_drone.header.stamp = ros::Time::now();
      odom_msg_drone.header.frame_id = "world";
      odom_msg_drone.child_frame_id = "drone";
      odom_msg_drone.pose.pose.position.x = x(0);
      odom_msg_drone.pose.pose.position.y = x(1);
      odom_msg_drone.pose.pose.position.z = x(2);
      odom_msg_drone.pose.pose.orientation.w = q(0);
      odom_msg_drone.pose.pose.orientation.x = q(1);
      odom_msg_drone.pose.pose.orientation.y = q(2);
      odom_msg_drone.pose.pose.orientation.z = q(3);
      pub_odom_drone.publish(odom_msg_drone);



      marker.header.stamp = ros::Time::now();
      marker.pose.position.x = x_vlp(0);
      marker.pose.position.y = x_vlp(1);
      marker.pose.position.z = x_vlp(2);
      marker.pose.orientation.w = q_vlp(0);
      marker.pose.orientation.x = q_vlp(1);
      marker.pose.orientation.y = q_vlp(2);
      marker.pose.orientation.z = q_vlp(3);
      pub_marker_lidar.publish( marker );

      marker_setpoint.header.stamp = ros::Time::now();
      marker_setpoint.pose.position.x = pref(0);
      marker_setpoint.pose.position.y = pref(1);
      marker_setpoint.pose.position.z = pref(2);
      marker_setpoint.pose.orientation.w = 1;
      marker_setpoint.pose.orientation.x = 0;
      marker_setpoint.pose.orientation.y = 0;
      marker_setpoint.pose.orientation.z = 0;
      pub_marker_setpoint.publish( marker_setpoint );




      loop_rate.sleep();


    }

}

