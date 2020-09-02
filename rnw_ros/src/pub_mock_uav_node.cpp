#include <ros/ros.h>
#include <Eigen/Geometry>

#include <rnw_ros/ConeState.h>
#include <uav_utils/utils.h>

ros::Publisher pub_uav_odom;

void on_cone_state( rnw_ros::ConeStateConstPtr const & msg ){
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "world";
  odom.pose.pose.orientation = uav_utils::to_quaternion_msg(Eigen::Quaterniond::Identity());
  odom.pose.pose.position = msg->tip;
  odom.pose.pose.position.z += 0.1;
  pub_uav_odom.publish(odom);
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  pub_uav_odom = nh.advertise<nav_msgs::Odometry>("odom",100);

  ros::Subscriber sub_odom = nh.subscribe<rnw_ros::ConeState>(
          "cone_state",
          10,
          on_cone_state,
          nullptr,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}