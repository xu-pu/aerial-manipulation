//
// Created by sheep on 2020/8/6.
//

#ifndef SRC_RNW_UTILS_H
#define SRC_RNW_UTILS_H

#include <deque>
#include <quadrotor_msgs/Float64Stamped.h>

#include "rnw_ros/ros_utils.h"
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RockingCmd.h>
#include <rnw_ros/RNWConfig.h>

struct rnw_config_t {

    Vector3d X_tip_body;

    Vector3d X_tcp_cage;

    double insert_below_tip;

    double ground_z;

    struct {

        double forward = 0;

        double downward = 0;

    } topple;

    struct {

        double step_forward = 0;

        double step_sideways = 0;

        size_t cycles = 0;

        double max_vel = 0;

        double max_acc = 0;

    } zigzag;

    struct {

        double radius;

        double apex;

        double height;

        Vector3d base_center;

        Vector3d tip;

    } cone;

    struct {

        double insertion_depth;

        double topple_init;

        double desired_nutation;

        double tau;

        double max_vel;

        double max_acc;

        double rocking_max_vel;

        double rocking_max_acc;

        double hover_above_tip;

        inline std::string to_string() const {
          std::stringstream ss;
          ss << "RnW Parameters:\n"
             << "insertion_depth: " << insertion_depth << "\n"
             << "topple_init: " << topple_init << "\n"
             << "desired_nutation: " << desired_nutation << "\n"
             << "max_vel: " << max_vel << "\n"
             << "max_acc: " << max_acc << "\n"
             << "rocking_max_vel: " << rocking_max_vel << "\n"
             << "rocking_max_acc: " << rocking_max_acc << "\n";
          return ss.str();
        }

        inline rnw_ros::RNWConfig to_config() const {
          rnw_ros::RNWConfig config;
          config.insertion_depth = insertion_depth;
          config.topple_init = topple_init;
          config.tau = tau;
          config.desired_nutation = desired_nutation;
          config.max_vel = max_vel;
          config.max_acc = max_acc;
          config.rocking_max_vel = rocking_max_vel;
          config.rocking_max_acc = rocking_max_acc;
          config.hover_above_tip = hover_above_tip;
          return config;
        }

    } rnw;

    void load_from_ros( ros::NodeHandle & nh );

};

/**
 * For downward moundted caging end-effector
 * Waypoints for initialization, insert -> topple
 * @param rnw_config
 * @return
 */
vector<Vector3d> gen_topple_waypoints_local( rnw_config_t const & rnw_config );

/**
 * x-psi-precession
 * y-theta-nutation (tilt)
 * z-phi-spin
 * @param R
 * @return - (x,y,z)
 */
Vector3d cone_rot2euler( Matrix3d const & R );

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
