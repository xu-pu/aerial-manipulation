//
// Created by sheep on 2020/8/6.
//

#ifndef SRC_RNW_UTILS_H
#define SRC_RNW_UTILS_H

#include <deque>
#include <quadrotor_msgs/Float64Stamped.h>
#include <rnw_msgs/ConeState.h>
#include <nav_msgs/Odometry.h>
#include <rnw_msgs/GripState.h>
#include <rnw_msgs/RockingCmd.h>

#include "rnw_ros/rnw_config.h"

/**
 * There are some bugs in the traj generator
 * - create mid points to make sure there is at least 3 waypoints
 * - remove waypoint too close together
 * @param wpts
 * @return
 */
bool check_waypoints( vector<Vector3d> & wpts );

vector<Vector3d> add_mid_points( vector<Vector3d> const & src );

inline bool cone_is_qstatic( rnw_msgs::ConeState const & cone_state, rnw_config_t const & cfg ){
  return cone_state.euler_angles.y > cfg.rnw.min_nutation_deg * deg2rad
         && cone_state.euler_angles_velocity.z <= cfg.rnw.ang_vel_threshold;
}

double uav_yaw_from_cone_state( rnw_msgs::ConeState const & cone_state );

double uav_yaw_from_odom( nav_msgs::Odometry const & odom );

/**
 * End-effector is mounted on the back, there is a 180 degree offset
 * @param cone_yaw
 * @return
 */
double uav_yaw_from_cone_yaw( double cone_yaw );

double cone_yaw( rnw_msgs::ConeState const & cone_state );

double calc_obj_heading( rnw_msgs::ConeState const & s1, rnw_msgs::ConeState const & s2 );

double calc_mid_rad( double r1, double r2 );

/**
 * For downward mounted caging end-effector
 * Waypoints for initialization, insert -> topple
 * @param rnw_config
 * @return
 */
vector<Vector3d> gen_wpts_insert_topple(rnw_config_t const & rnw_config );

/**
 * For forward mounted open cage, no insertion
 * waypoints are for the UAV in the world frame
 * @param rnw_config
 * @return
 */
vector<Vector3d> gen_wpts_push_topple(
        nav_msgs::Odometry const & uav_odom,
        rnw_msgs::ConeState const & cone_state,
        rnw_config_t const & rnw_config
);

/**
 * x-psi-precession
 * y-theta-nutation (tilt)
 * z-phi-spin
 * @param R
 * @return - (x,y,z) - rad
 */
Vector3d cone_rot2euler( Matrix3d const & R );

/**
 * Distance between 3d line AB and 3d point C
 * See https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
 * @param pt1 - A
 * @param pt2 - B
 * @param pt - C
 * @return distance
 */
double line_point_dist_3d( Vector3d const & A, Vector3d const & B, Vector3d const & C );

/**
 * Rodrigues' Rotation Formula
 * see https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 * rotate point X along axis (O+lambda*K), right-hand-rule theta
 * @param X - the point to rotate
 * @param O - origin of the rotation axis
 * @param K - unit normal vector of the rotation axis
 * @param theta - radiant
 * @return
 */
Vector3d rotate_point_along_axis( Vector3d const & X, Vector3d const & O, Vector3d const & K, double theta );

double dist( Vector3d const & A, Vector3d const & B );

inline double square( double val ){ return val*val; }

struct grip_state_t {

    grip_state_t();

    /**
     * Calculate the current grip state
     * @param cone_state
     * @param uav_odom
     * @param flu_T_tcp
     */
    grip_state_t(
            rnw_msgs::ConeState const & cone_state,
            nav_msgs::Odometry const & uav_odom,
            Vector3d const & flu_T_tcp
    );

    rnw_msgs::ConeState cone_state;

    nav_msgs::Odometry uav_odom;

    Vector3d flu_T_tcp;

    Vector3d grip_point;

    bool grip_valid = false;

    double grip_radius = 0;

    double grip_depth = 0;

    bool initialized = false;

    rnw_msgs::GripState to_msg() const;

};

/**
 * Transform tip position to UAV position,
 * using downward mounted caging end-effector
 * @param tip
 * @param config
 * @return
 */
Vector3d tip_position_to_uav_position( Vector3d const & tip, rnw_config_t const & config );

/**
 * Desired TCP position to desired UAV position while keep current rotation
 * @param tcp
 * @param uav_odom
 * @param flu_T_tcp
 * @return
 */
Vector3d tcp2uav( Vector3d const & tcp, nav_msgs::Odometry const & uav_odom, Vector3d const & flu_T_tcp );

/**
 * Calculate the desired TCP position under current cone_state at specific grip_depth
 * @param cone_state
 * @param grip_depth
 * @return tcp position
 */
Vector3d point_at_grip_depth( rnw_msgs::ConeState const & cone_state, double grip_depth );

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
