//
// Created by sheep on 2020/8/6.
//

#ifndef SRC_RNW_UTILS_H
#define SRC_RNW_UTILS_H

#include <deque>
#include <quadrotor_msgs/Float64Stamped.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <rnw_msgs/ConeState.h>
#include <nav_msgs/Odometry.h>
#include <rnw_msgs/GripState.h>
#include <rnw_msgs/RockingCmd.h>

#include "rnw_ros/rnw_config.h"

Eigen::Vector3d action_to_cmd_vel( rnw_msgs::ConeState const & cone_state, Eigen::Vector2d const & action );

Eigen::Matrix3d calc_rnw_body_frame( rnw_msgs::ConeState const & );

Matrix3d intermediate_rotation( nav_msgs::Odometry const & );

Vector3d point_in_intermediate_frame( Vector3d const & point, nav_msgs::Odometry const & frame );

vector<Vector3d> points_in_intermediate_frame( vector<Vector3d> const & points, nav_msgs::Odometry const & frame );

quadrotor_msgs::PolynomialTrajectory gen_setpoint_traj(nav_msgs::Odometry const & odom, Vector3d const & setpoint, double yaw, double duration );

quadrotor_msgs::PolynomialTrajectory gen_setpoint_traj(nav_msgs::Odometry const & odom, Vector3d const & setpoint, double duration );

inline double get_traj_duration( quadrotor_msgs::PolynomialTrajectory const & msg ){
  double rst = 0;
  for ( double i : msg.time ) {
    rst += i;
  }
  return rst;
}

/**
 * Calculate the desired drone position given control point frame and cable length and direction
 * @param CP - control point
 * @param heading - heading direction (yaw)
 * @param cable - cable length
 * @param rad - cable angle (rotation along x axis)
 * @return
 */
inline Vector3d calc_pt_at_cp_frame( Vector3d const & CP, double heading, double cable, double rad ){
  // reference frame at the control point
  Matrix3d R = Eigen::AngleAxisd(heading,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d pt(0,cable*std::sin(rad),cable*cos(rad));
  return R*pt+CP;
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

double calc_cone_heading_direction( rnw_msgs::ConeState const & cone_state );

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

/**
 * @param cone_state
 * @param point - world frame
 * @param nutation_rad
 * @return
 */
Vector3d point_at_nutation( rnw_msgs::ConeState const & cone_state, Vector3d const & point, double nutation_rad );

double calc_mechanical_energy( rnw_msgs::ConeState const & cone_state, double mass, double xCM, double zCM );

double calc_kinetic_energy( rnw_msgs::ConeState const & cone_state, double mass, double xCM, double zCM );

double calc_potential_energy( rnw_msgs::ConeState const & cone_state, double mass, double xCM, double zCM );

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

/**
 * Butterworth First-Order Low Pass Filter
 */
struct lpf_1st_butterworth_t {

    bool init = false;

    double last_value = 0;

    double T = 1;

    inline lpf_1st_butterworth_t() = default;

    inline explicit lpf_1st_butterworth_t( double param ) : T(param) { }

    inline double filter( double measurement ){
      if ( !init ) {
        init = true;
        last_value = measurement;
        return last_value;
      }
      else {
        last_value += (measurement-last_value)/(1+(1/(2.*M_PI*T)));
        return last_value;
      }
    }

};


#endif //SRC_RNW_UTILS_H
