#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/cone_state_estimator.h"

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  cone_state_estimator_t cone_state_estimator(nh);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "/odom/cone",
          10,
          &cone_state_estimator_t::on_odom,
          &cone_state_estimator,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}