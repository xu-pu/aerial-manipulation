#include <ros/ros.h>
#include <Eigen/Geometry>

#include "rnw_ros/rnw_utils.h"

#include <rnw_msgs/ConeState.h>
#include <uav_utils/utils.h>

ros::Publisher pub_uav_odom;

rnw_config_t rnw_config;

void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "world";
  odom.pose.pose.orientation = uav_utils::to_quaternion_msg(Eigen::Quaterniond::Identity());
  Vector3d tcp = point_at_grip_depth(*msg,rnw_config.rnw.desired_grip_depth);
  Vector3d uav = tcp - rnw_config.flu_T_tcp;
  odom.pose.pose.position = uav_utils::to_point_msg(uav);
  pub_uav_odom.publish(odom);
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  rnw_config.load_from_ros(nh);

  pub_uav_odom = nh.advertise<nav_msgs::Odometry>("/odom/uav",100);

  ros::Subscriber sub_odom = nh.subscribe<rnw_msgs::ConeState>(
          "/rnw/cone_state",
          10,
          on_cone_state,
          nullptr,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}