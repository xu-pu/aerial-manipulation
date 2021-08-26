#include <ros/ros.h>

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/pose_utils.h"

string tf_frame_name, tf_parent_name;

void pose2tf( geometry_msgs::Pose const & pose, string const & name, string const & parent ){
  publish_frame( pose2R(pose), pose2T(pose), name, parent );
}

void on_odom( nav_msgs::OdometryConstPtr const & msg ){
  pose2tf(msg->pose.pose,tf_frame_name,tf_parent_name);
}

void on_pose( geometry_msgs::PoseStampedConstPtr const & msg ){
  pose2tf(msg->pose,tf_frame_name,tf_parent_name);
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"odom2tf_node");

  ros::NodeHandle nh("~");

  tf_frame_name = get_param_default<string>(nh,"name","name");
  tf_parent_name = get_param_default<string>(nh,"parent","world");

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "odom",
          1,
          on_odom,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
          "pose",
          1,on_pose,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}