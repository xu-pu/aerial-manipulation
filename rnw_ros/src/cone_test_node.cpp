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

Vector3d X_top_point;

void on_pose( const geometry_msgs::PoseStamped::ConstPtr msg ){

  Matrix3d R = pose2R(msg->pose);
  Vector3d T = pose2T(msg->pose);

  Vector3d tip = R*X_top_point+T;

  ROS_INFO_STREAM("Tip = " << tip.transpose());

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"cone_test_node");

  ros::NodeHandle nh("~");

  X_top_point = { -0.01, 0, 0.82 };

  ros::Subscriber sub_odom = nh.subscribe<geometry_msgs::PoseStamped>("/cone/posose",10,on_pose);

  ros::spin();

  ros::shutdown();

  return 0;

}