#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/cone_state_estimator.h"

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  cone_state_estimator_t cone_state_estimator(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}