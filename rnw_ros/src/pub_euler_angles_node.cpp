#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

ros::Publisher pub_rpy_imu;
ros::Publisher pub_rpy_odom;

void on_odom( OdometryConstPtr const & odom ){
  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  Vector3d rpy = odom2rpy(odom);
  pub_rpy_odom.publish(eigen2ros(rpy));
}

void on_imu( sensor_msgs::ImuConstPtr const & imu ){
  Vector3d rpy = imu2rpy(imu);
  pub_rpy_imu.publish(eigen2ros(rpy));
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_euler_angles_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu",10,on_imu);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("uav",10,on_odom);

  pub_rpy_imu = nh.advertise<geometry_msgs::Vector3>("/rpy/imu",1);
  pub_rpy_odom = nh.advertise<geometry_msgs::Vector3>("/rpy/odom",1);

  ros::spin();

  ros::shutdown();

  return 0;

}