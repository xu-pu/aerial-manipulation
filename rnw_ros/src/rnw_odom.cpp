#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

using namespace std;

using geometry_msgs::PoseStamped;
using geometry_msgs::PoseStampedConstPtr;
using geometry_msgs::TwistStamped;
using geometry_msgs::TwistStampedConstPtr;
using geometry_msgs::AccelStamped;
using geometry_msgs::AccelStampedConstPtr;

ros::Publisher pub_pose;
ros::Publisher pub_uav_path;

void publish_frame( Matrix3d const & R, Vector3d const & T, string const & name, string const & parent ){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(T.x(),T.y(),T.z()) );
  Eigen::Quaterniond quat(R);
  tf::Quaternion q(quat.x(),quat.y(),quat.z(),quat.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
}

void optitrack_callback( PoseStampedConstPtr const & pose, sensor_msgs::ImuConstPtr const & imu ){

  static nav_msgs::Path path;

  ROS_INFO("got it");
  
  auto const & q1 = imu->orientation;
  auto const & q2 = pose->pose.orientation;
  auto const & t2 = pose->pose.position;

  Quaterniond qimu(q1.w,q1.x,q1.y,q1.z);
  Quaterniond qvicon(q2.w,q2.x,q2.y,q2.z);

  Vector3d tvicon(t2.x,t2.y,t2.z);

  Matrix3d Rwv = qimu.toRotationMatrix() * qvicon.toRotationMatrix().transpose();

  Vector3d Twb = Rwv * tvicon;

  publish_frame(qimu.toRotationMatrix(),Twb,"uav","world");
  publish_frame(Rwv,Vector3d::Zero(),"vicon","world");
  publish_frame(qvicon.toRotationMatrix(),tvicon,"vicon_pose","vicon");

  nav_msgs::Odometry msg;
  msg.header.frame_id = "world";
  msg.header.stamp = pose->header.stamp;
  msg.pose.pose.orientation = q1;
  msg.pose.pose.position.x = Twb.x();
  msg.pose.pose.position.y = Twb.y();
  msg.pose.pose.position.z = Twb.z();
  pub_pose.publish(msg);

  path.header.frame_id = "world";
  path.poses.push_back(*pose);
  pub_uav_path.publish(path);

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_odom_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<PoseStamped> sub_vicon(nh, "/vrpn_client_node/SwarnDrone01/pose", 10);
  message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, "/dji_sdk_1/dji_sdk/imu", 10);

  typedef message_filters::sync_policies::ApproximateTime<PoseStamped,sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10),sub_vicon,sub_imu);
  sync.registerCallback(optitrack_callback);

  pub_pose = nh.advertise<nav_msgs::Odometry>("/rnw/vicon_odom",10);
  pub_uav_path = nh.advertise<nav_msgs::Path>("/rnw/uav_path",10);

  ros::spin();

  ros::shutdown();

  return 0;

}