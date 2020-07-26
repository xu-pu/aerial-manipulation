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

ros::Publisher pub_rpy_imu;
ros::Publisher pub_rpy_odom;
ros::Publisher pub_cone_tip;
Vector3d last_t;
Vector3d X_tip_body;

void on_odom( OdometryConstPtr const & odom ){
  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  Vector3d rpy = odom2rpy(odom);
  pub_rpy_odom.publish(eigen2ros(rpy));
  publish_frame(odom2R(odom),odom2T(odom),"uav_odom","world");
  last_t = odom2T(odom);
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

  X_tip_body.x() = get_param_default(nh,"X_tip_body/x",0.);
  X_tip_body.y() = get_param_default(nh,"X_tip_body/y",0.);
  X_tip_body.z() = get_param_default(nh,"X_tip_body/z",0.);

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu",10,on_imu);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("uav",10,on_odom);
  ros::Subscriber sub_cone = nh.subscribe<nav_msgs::Odometry>("cone",10,on_cone);
  ros::Subscriber sub_vins = nh.subscribe<nav_msgs::Odometry>("vins",10,on_vins);

  pub_rpy_imu = nh.advertise<geometry_msgs::Vector3>("/uav/rpy_imu",1);
  pub_rpy_odom = nh.advertise<geometry_msgs::Vector3>("/uav/rpy_odom",1);
  pub_cone_tip = nh.advertise<geometry_msgs::PointStamped>("/cone/tip",1);

  ros::spin();

  ros::shutdown();

  return 0;

}