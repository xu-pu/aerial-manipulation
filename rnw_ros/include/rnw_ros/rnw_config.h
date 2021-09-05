//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_RNW_CONFIG_H
#define SRC_RNW_CONFIG_H

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/RNWConfig.h"
#include "rnw_ros/RlConfig.h"

struct rnw_config_t {

    Vector3d flu_T_tcp;

    double ground_z;

    struct {

        double step_forward = 0;

        double step_sideways = 0;

        size_t cycles = 0;

        double max_vel = 0;

        double max_acc = 0;

    } zigzag;

    struct {

        double radius;

        Vector3d base_center;

        Vector3d tip;

        double CoM_x;

        double CoM_z;

        double mass;

        inline Vector3d CoM() const {
          Vector3d rst = base_center;
          rst.x() = rst.x() + CoM_x;
          rst.z() = rst.z() + CoM_z;
          return rst;
        }

        inline double height() const {
          return tip.z() - base_center.z();
        }

    } cone;

    struct {

        /**
         * initialization
         */

        double init_threshold; // consider initialized when peak_phi_dot > init_threshold

        double init_tau;

        double init_ang_vel_threshold;

        double init_min_step_interval;

        double phi_epsi;

        /**
         * after initialization
         */

        double tau;

        double ang_vel_threshold;

        double min_step_interval;

        /**
         * energy regulator
         */

        bool enable_energy_feedback = true;

        bool specify_energy = false;

        double desired_energy = 1;

        double EKp = 0;

        double EKi = 0;

        double EKd = 0;

        /**
         * precession regulator
         */

        bool enable_steering = true;

        bool specify_heading = false;

        double heading;

        double yaw_gain;

        /**
         * posture regulator
         */

        double desired_nutation;

        double min_nutation_deg;

        /**
         * drone motion
         */

        bool direct_control;

        double rocking_max_vel;

        double rocking_max_acc;

        /*********************************/

        double insertion_depth;

        double lap_ang_vel_deg;

        size_t lap_start;

        inline rnw_ros::RNWConfig to_config() const {
          rnw_ros::RNWConfig config;

          config.tau = tau;
          config.ang_vel_threshold = ang_vel_threshold;
          config.min_step_interval = min_step_interval;

          config.phi_epsi = rad2deg * phi_epsi;
          config.init_threshold = init_threshold;
          config.init_tau = init_tau;
          config.init_ang_vel_threshold = init_ang_vel_threshold;
          config.init_min_step_interval = init_min_step_interval;

          config.enable_energy_feedback = enable_energy_feedback;
          config.desired_energy = desired_energy;
          config.EKp = EKp;
          config.EKi = EKi;
          config.specify_energy = specify_energy;
          config.desired_energy = desired_energy;

          config.desired_nutation = desired_nutation;
          config.rocking_max_vel = rocking_max_vel;
          config.rocking_max_acc = rocking_max_acc;
          config.direct_control = direct_control;

          return config;
        }

    } rnw;

    struct {
        double angle;
    } swarm;

    struct {
        double drone1;
        double drone2;
    } cable;

    struct {
        double desired_grip_depth;
    } caging;

    struct {

        bool enable_x = true;

        bool enable_y = true;

        double action_scale = 0.5;

        double yaw = M_PI_2;

        inline void set( rnw_ros::RlConfig const & cfg ){
          enable_x = cfg.enable_x;
          enable_y = cfg.enable_y;
          action_scale = cfg.action_scale;
        }

        inline rnw_ros::RlConfig to_config() const {
          rnw_ros::RlConfig config;
          config.action_scale = action_scale;
          config.enable_x = enable_x;
          config.enable_y = enable_y;
          return config;
        }

    } rl;

    void load_from_ros( ros::NodeHandle & nh );

};


#endif //SRC_RNW_CONFIG_H
