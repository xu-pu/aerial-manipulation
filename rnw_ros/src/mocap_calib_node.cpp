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

Matrix3d R;
Vector3d T;

void on_pose( PoseStampedConstPtr const & msg ){
  R = pose2R(msg->pose);
  T = pose2T(msg->pose);
}

void on_point( geometry_msgs::PointConstPtr const & msg ){
  // OptiTrack's cord is in (x,z,-y)
  Vector3d X_vicon ( msg->x, -msg->z, msg->y );
  Vector3d X_body = R.transpose() * ( X_vicon - T );
  cout << "Tip point in body frame (yaml)\n"
       << "point:\n"
       << "   x: " << X_body.x() << endl
       << "   y: " << X_body.y() << endl
       << "   z: " << X_body.z() << endl
       << endl;
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"body_point_calib_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_odom = nh.subscribe<geometry_msgs::PoseStamped>("body",10,on_pose);
  ros::Subscriber sub_point = nh.subscribe<geometry_msgs::Point>("point",10,on_point);

  ROS_INFO_STREAM("Send coordinates through command line:");
  ROS_INFO_STREAM("rostopic pub /cmd/point geometry_msgs/Point '{ x: 1, y: 2, z: 3 }'");
  ROS_INFO_STREAM("x, y, z is in the order OptiTrack shows, we will handle it");

  ros::spin();

  ros::shutdown();

  return 0;

}