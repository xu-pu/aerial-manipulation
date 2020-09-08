#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/cone_state_estimator.h"

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  cone_state_estimator_t cone_state_estimator(nh);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "odom",
          10,
          &cone_state_estimator_t::on_odom,
          &cone_state_estimator,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Rate rate(80);

  nav_msgs::OdometryPtr odom = boost::make_shared<nav_msgs::Odometry>();
  odom->pose.pose.orientation = uav_utils::to_quaternion_msg(Quaterniond::Identity());

  while ( ros::ok() ) {

    odom->header.stamp = ros::Time::now();

    cone_state_estimator.on_odom(odom);

    rate.sleep();

  }

  ros::shutdown();

  return 0;

}