//
// Created by sheep on 2020/6/5.
//
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

void odom_callback( nav_msgs::OdometryConstPtr const & msg ){

  auto const & T = msg->pose.pose.position;
  auto const & quat = msg->pose.pose.orientation;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(T.x,T.y,T.z) );
  tf::Quaternion q(quat.x,quat.y,quat.z,quat.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uwb"));

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_tf_node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/uwb_vicon_odom",1,odom_callback);

  ros::spin();

  return 0;

}