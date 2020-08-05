//
// Created by sheep on 2020/7/26.
//

#ifndef SRC_ROS_UTILS_H
#define SRC_ROS_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

using namespace std;

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
