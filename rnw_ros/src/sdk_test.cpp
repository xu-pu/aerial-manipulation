#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>

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

using nav_msgs::OdometryConstPtr;
using sensor_msgs::ImuConstPtr;

ros::Publisher pub_rpy_imu;
ros::Publisher pub_rpy_odom;

Vector3d last_t;

void publish_frame( Matrix3d const & R, Vector3d const & T, string const & name, string const & parent ){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(T.x(),T.y(),T.z()) );
  Eigen::Quaterniond quat(R);
  tf::Quaternion q(quat.x(),quat.y(),quat.z(),quat.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
}

inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat) {
  Eigen::Vector3d rpy;
  rpy.x() = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()),
                  1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
  rpy.y() = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
  rpy.z() = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                  1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()));
  return rpy;
}

inline Eigen::Matrix3d rpy2rot( Vector3d const & rpy ){
  return Eigen::AngleAxisd(rpy.z(), Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Vector3d::UnitX())
                 .toRotationMatrix();
}

void static_test(){
  Vector3d angles(0.1,0.2,0.3);
  Matrix3d rot = rpy2rot(angles);
  Vector3d back = quat2eulers(Quaterniond(rot));
  cout << back.transpose() << endl;
}

Vector3d imu2rpy( ImuConstPtr const & imu ){

  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  auto const & quat = imu->orientation;

  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Eigen::Matrix3d RFLU2ENU = q.toRotationMatrix();

  q = Eigen::Quaterniond(R_ENU2NED*RFLU2ENU*R_FLU2FRD.transpose());
  return quat2eulers(q);

}

Vector3d odom2rpy( OdometryConstPtr const & odom ){

  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  auto const & quat = odom->pose.pose.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Matrix3d FLU2NED = R_ENU2NED*q.toRotationMatrix()*R_FLU2FRD.transpose();
  return quat2eulers(Quaterniond(FLU2NED));

}

Matrix3d odom2R( OdometryConstPtr const & odom ){
  auto const & quat = odom->pose.pose.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  return q.toRotationMatrix();
}

Vector3d odom2T( OdometryConstPtr const & odom ){
  auto const & pos = odom->pose.pose.position;
  return { pos.x, pos.y, pos.z };
}

Matrix3d imu2R( ImuConstPtr const & imu ){
  auto const & quat = imu->orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  return q.toRotationMatrix();
}

geometry_msgs::Vector3 eigen2ros( Vector3d const & src ){
  geometry_msgs::Vector3 dst;
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  return dst;
}

void on_odom( OdometryConstPtr const & odom ){
  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  Vector3d rpy = odom2rpy(odom);
  pub_rpy_odom.publish(eigen2ros(rpy));
  publish_frame(odom2R(odom),odom2T(odom),"vicon","world");
  publish_frame(Matrix3d::Identity(),odom2T(odom),"vicon_ref","world");
  last_t = odom2T(odom);
}

void on_vins( OdometryConstPtr const & odom ){
  Vector3d rpy = odom2rpy(odom);
  pub_rpy_odom.publish(eigen2ros(rpy));
  publish_frame(odom2R(odom),odom2T(odom),"vins","world");
  last_t = odom2T(odom);
}

void on_imu( sensor_msgs::ImuConstPtr const & imu ){
  Vector3d rpy = imu2rpy(imu);
  pub_rpy_imu.publish(eigen2ros(rpy));
  Matrix3d R = imu2R(imu);
  publish_frame(R,last_t,"imu","world");
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"sdk_test_node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("/djiros/imu",10,on_imu);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/uwb_vicon_odom",10,on_odom);
  ros::Subscriber sub_vins = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate",10,on_vins);
  ros::Subscriber sub_local = nh.subscribe<nav_msgs::Odometry>("/pos_vel_mocap/odom_TA",10,on_odom);

  pub_rpy_imu = nh.advertise<geometry_msgs::Vector3>("/uav/rpy_imu",1);
  pub_rpy_odom = nh.advertise<geometry_msgs::Vector3>("/uav/rpy_odom",1);

  ros::spin();

  ros::shutdown();

  return 0;

}