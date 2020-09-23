//
// Created by sheep on 2020/7/6.
//

#ifndef SRC_POSE_UTILS_H
#define SRC_POSE_UTILS_H

#include "rnw_ros/ros_utils.h"

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using geometry_msgs::PoseStamped;
using geometry_msgs::PoseStampedConstPtr;
using nav_msgs::OdometryConstPtr;
using sensor_msgs::ImuConstPtr;
using geometry_msgs::PoseStampedConstPtr;

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

inline Vector3d imu2rpy( sensor_msgs::Imu const & imu ){

  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  auto const & quat = imu.orientation;

  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Eigen::Matrix3d RFLU2ENU = q.toRotationMatrix();

  q = Eigen::Quaterniond(R_ENU2NED*RFLU2ENU*R_FLU2FRD.transpose());
  return quat2eulers(q);

}

inline Vector3d imu2rpy( ImuConstPtr const & imu ){
  return imu2rpy(*imu);
}

inline Matrix3d pose2R( geometry_msgs::Pose const & pose ){
  auto const & quat = pose.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  return q.normalized().toRotationMatrix();
}

inline Vector3d pose2T( geometry_msgs::Pose const & pose ){
  auto const & pos = pose.position;
  return { pos.x, pos.y, pos.z };
}

inline Vector3d odom2rpy( OdometryConstPtr const & odom ){

  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  auto const & quat = odom->pose.pose.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  Matrix3d FLU2NED = R_ENU2NED*q.toRotationMatrix()*R_FLU2FRD.transpose();
  return quat2eulers(Quaterniond(FLU2NED));

}

inline Vector3d odom2rpy( Quaterniond const & odom ){

  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;

  Matrix3d FLU2NED = R_ENU2NED*odom.toRotationMatrix()*R_FLU2FRD.transpose();
  return quat2eulers(Quaterniond(FLU2NED));

}

inline Matrix3d odom2R( OdometryConstPtr const & odom ){
  return pose2R(odom->pose.pose);
}

inline Matrix3d odom2R( nav_msgs::Odometry const & odom ){
  return pose2R(odom.pose.pose);
}

inline Vector3d odom2T( nav_msgs::Odometry const & odom ){
  return pose2T(odom.pose.pose);
}

inline Vector3d odom2T( OdometryConstPtr const & odom ){
  return pose2T(odom->pose.pose);
}

inline Matrix3d imu2R( ImuConstPtr const & imu ){
  auto const & quat = imu->orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  return q.toRotationMatrix();
}

inline void publish_frame( Matrix3d const & R, Vector3d const & T, string const & name, string const & parent ){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(T.x(),T.y(),T.z()) );
  Eigen::Quaterniond quat(R);
  tf::Quaternion q(quat.x(),quat.y(),quat.z(),quat.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
}

#endif //SRC_POSE_UTILS_H
