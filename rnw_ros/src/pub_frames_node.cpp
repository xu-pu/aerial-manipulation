#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

ros::Publisher pub_rpy_imu;
ros::Publisher pub_rpy_odom;

Vector3d last_t;


void static_test(){
  Vector3d angles(0.1,0.2,0.3);
  Matrix3d rot = rpy2rot(angles);
  Vector3d back = quat2eulers(Quaterniond(rot));
  cout << back.transpose() << endl;
}


void on_odom( OdometryConstPtr const & odom ){
  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  Vector3d rpy = odom2rpy(odom);
  pub_rpy_odom.publish(eigen2ros(rpy));
  publish_frame(odom2R(odom),odom2T(odom),"vicon","world");
  publish_frame(Matrix3d::Identity(),odom2T(odom),"vicon_ref","world");
  last_t = odom2T(odom);
}

void on_vins( OdometryConstPtr const & odom ){
  Vector3d rpy = odom2rpy(odom);
  pub_rpy_odom.publish(eigen2ros(rpy));
  publish_frame(odom2R(odom),odom2T(odom),"vins","world");
  last_t = odom2T(odom);
}

void on_imu( sensor_msgs::ImuConstPtr const & imu ){
  Vector3d rpy = imu2rpy(imu);
  pub_rpy_imu.publish(eigen2ros(rpy));
  Matrix3d R = imu2R(imu);
  publish_frame(R,last_t,"imu","world");
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_frames_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu",10,on_imu);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("vicon",10,on_odom);
  ros::Subscriber sub_vins = nh.subscribe<nav_msgs::Odometry>("vins",10,on_vins);

  pub_rpy_imu = nh.advertise<geometry_msgs::Vector3>("/uav/rpy_imu",1);
  pub_rpy_odom = nh.advertise<geometry_msgs::Vector3>("/uav/rpy_odom",1);

  ros::spin();

  ros::shutdown();

  return 0;

}