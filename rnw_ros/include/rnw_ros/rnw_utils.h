//
// Created by sheep on 2020/8/6.
//

#ifndef SRC_RNW_UTILS_H
#define SRC_RNW_UTILS_H

#include <deque>
#include <quadrotor_msgs/Float64Stamped.h>
#include <rnw_msgs/ConeState.h>
#include <nav_msgs/Odometry.h>

#include "rnw_ros/rnw_config.h"

/**
 * For downward moundted caging end-effector
 * Waypoints for initialization, insert -> topple
 * @param rnw_config
 * @return
 */
vector<Vector3d> gen_wpts_insert_topple(rnw_config_t const & rnw_config );

vector<Vector3d> gen_wpts_push_topple( rnw_config_t const & rnw_config );

/**
 * x-psi-precession
 * y-theta-nutation (tilt)
 * z-phi-spin
 * @param R
 * @return - (x,y,z)
 */
Vector3d cone_rot2euler( Matrix3d const & R );

struct grip_state_t {

    rnw_msgs::ConeState cone_state;

    nav_msgs::Odometry uav_odom;

    Vector3d const & flu_T_grip;

    bool grip_valid;

    double grip_radius;

    double grip_depth;

};

grip_state_t calc_gripping_point(
        rnw_msgs::ConeState const & cone_state,
        nav_msgs::Odometry uav_odom,
        Vector3d const & flu_T_grip
);

/**
 * Transform tip position to UAV position,
 * using downward mounted caging end-effector
 * @param tip
 * @param config
 * @return
 */
Vector3d tip_position_to_uav_position( Vector3d const & tip, rnw_config_t const & config );

template<typename T, size_t window_size>
struct median_filter_t {

    std::deque<T> values;

    median_filter_t() = default;

    T update( T val ){

      values.push_back(val);

      if ( values.size() > window_size ) {
        values.pop_front();
      }

      std::deque<T> sorted(values);
      std::sort(sorted.begin(),sorted.end());

      T output = sorted.at(sorted.size()/2);

      //ROS_INFO_STREAM("[median filter] input=" << val << ", output=" << output );

      return output;

    }

};

template<typename T, size_t window_size>
struct average_filter_t {

    std::deque<T> values;

    average_filter_t() = default;

    T update( T val ){

      values.push_back(val);

      if ( values.size() > window_size ) {
        values.pop_front();
      }

      T sum = 0;

      for ( auto v : values ) { sum+=v; }

      return sum/values.size();

    }

};

#endif //SRC_RNW_UTILS_H
