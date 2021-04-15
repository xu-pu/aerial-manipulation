#include <ros/ros.h>

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/pose_utils.h"

string tf_frame_name, tf_parent_name;

void on_odom( nav_msgs::OdometryConstPtr const & msg ){
  publish_frame(odom2R(msg),odom2T(msg),tf_frame_name,tf_parent_name);
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"odom2tf_node");

  ros::NodeHandle nh("~");

  tf_parent_name = get_param_default<string>(nh,"name","name");
  tf_frame_name = get_param_default<string>(nh,"parent","world");

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("odom",1,on_odom);

  ros::spin();

  ros::shutdown();

  return 0;

}