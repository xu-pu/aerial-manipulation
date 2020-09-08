//
// Created by sheep on 2020/7/26.
//

#ifndef SRC_ROS_UTILS_H
#define SRC_ROS_UTILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <uav_utils/geometry_utils.h>
#include <uav_utils/converters.h>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

using namespace std;

template<typename Derived>
double msg_time_diff( Derived const & from, Derived const & to ){
  return (to.header.stamp - from.header.stamp).toSec();
}

template<typename T>
T get_param_default( ros::NodeHandle & nh, string const & key, T const & default_val ){
  T val;
  if ( !nh.getParam(key,val) ) {
    val = default_val;
    ROS_ERROR_STREAM("Parameter \'" << key << "\' not found, using default value " << default_val << "!");
  }
  return val;
}

#endif //SRC_ROS_UTILS_H
