#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

ros::Publisher pub_cone_tip;
ros::Publisher pub_cage_tcp;
Vector3d last_t;
Vector3d X_tip_body;
Vector3d X_tcp_cage;

Matrix3d calc_intermediate_R( double yaw ){

  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  Vector3d yaw_only = Vector3d::Zero();
  yaw_only.z() = yaw;
  Matrix3d R = rpy2rot(yaw_only);

  return R_ENU2NED.transpose() * R * R_FLU2FRD;

}


void on_odom( OdometryConstPtr const & odom ){
  Vector3d rpy = odom2rpy(odom);
  Matrix3d R = odom2R(odom);
  Vector3d T = odom2T(odom);
  publish_frame(R,T,"uav_odom","world");
  last_t = T;

  Matrix3d R_intermediate = calc_intermediate_R(rpy.z());

  Vector3d X_tcp = R_intermediate * X_tcp_cage + T;
  //Vector3d X_tcp = R * X_tcp_cage + T;
  geometry_msgs::PointStamped tcp_msg;
  tcp_msg.header = odom->header;
  tcp_msg.point.x = X_tcp.x();
  tcp_msg.point.y = X_tcp.y();
  tcp_msg.point.z = X_tcp.z();
  pub_cage_tcp.publish(tcp_msg);

}

void on_cone( OdometryConstPtr const & odom ){
  Matrix3d R = odom2R(odom);
  Vector3d T = odom2T(odom);
  publish_frame(R,T,"cone_odom","world");
  Vector3d tip = R * X_tip_body + T;
  geometry_msgs::PointStamped tip_msg;
  tip_msg.header = odom->header;
  tip_msg.point.x = tip.x();
  tip_msg.point.y = tip.y();
  tip_msg.point.z = tip.z();
  pub_cone_tip.publish(tip_msg);
}

void on_vins( OdometryConstPtr const & odom ){
  publish_frame(odom2R(odom),odom2T(odom),"vins","world");
  last_t = odom2T(odom);
}

void on_imu( sensor_msgs::ImuConstPtr const & imu ){
  Matrix3d R = imu2R(imu);
  publish_frame(R,last_t,"imu","world");
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_frames_node");

  ros::NodeHandle nh("~");

  X_tip_body.x() = get_param_default(nh,"X_tip_body/x",0.);
  X_tip_body.y() = get_param_default(nh,"X_tip_body/y",0.);
  X_tip_body.z() = get_param_default(nh,"X_tip_body/z",0.);

  X_tcp_cage.x() = get_param_default(nh,"X_tcp_cage/x",0.);
  X_tcp_cage.y() = get_param_default(nh,"X_tcp_cage/y",0.);
  X_tcp_cage.z() = get_param_default(nh,"X_tcp_cage/z",0.);

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu",10,on_imu);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("uav",10,on_odom);
  ros::Subscriber sub_cone = nh.subscribe<nav_msgs::Odometry>("cone",10,on_cone);
  ros::Subscriber sub_vins = nh.subscribe<nav_msgs::Odometry>("vins",10,on_vins);

  pub_cone_tip = nh.advertise<geometry_msgs::PointStamped>("/cone/tip",1);
  pub_cage_tcp = nh.advertise<geometry_msgs::PointStamped>("/uav/tcp",1);

  ros::spin();

  ros::shutdown();

  return 0;

}