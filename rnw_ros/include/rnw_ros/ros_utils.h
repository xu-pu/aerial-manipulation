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

constexpr double deg2rad = M_PI/180.;
constexpr double rad2deg = 180./M_PI;

template<typename T>
bool message_in_time( T msg, double sec ){
  return ( ros::Time::now() - msg.header.stamp ).toSec() < sec;
}

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


template<typename T>
T get_ros_param_required( ros::NodeHandle & nh, string const & key ){
  T val;
  if ( !nh.getParam(key,val) ) {
    ROS_FATAL_STREAM("Parameter \'" << key << "\' is required but not found, can not continue!");
    ROS_BREAK();
  }
  return val;
}


template<typename T>
double sec_since_msg( T const & msg ){
  return (ros::Time::now() - msg.header.stamp).toSec();
}

inline double sec_since( ros::Time const & t ){
  return (ros::Time::now() - t).toSec();
}

template<typename T>
struct sliding_window_t {

    explicit sliding_window_t( size_t sz_ ): sz(sz_) { }

    size_t size() const { return sz; }

    void push_back( T const & item ){
      data.template emplace_back(item);
      while ( data.size() <= sz ) {
        data.pop_front();
      }
    }

    bool empty() const  { return data.empty(); }

    T & back() { return data.back(); }

private:

    size_t sz;

    std::deque<T> data;

};

#endif //SRC_ROS_UTILS_H
