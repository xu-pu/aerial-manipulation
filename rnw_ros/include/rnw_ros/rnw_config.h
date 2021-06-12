//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_RNW_CONFIG_H
#define SRC_RNW_CONFIG_H

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/RNWConfig.h"

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

        bool direct_control;

        bool specify_heading = false;

        double heading;

        double insertion_depth;

        double desired_nutation;

        double tau;

        double min_step_interval;

        double waiting_ratio;

        double rocking_max_vel;

        double rocking_max_acc;

        double ang_vel_threshold;

        double min_nutation_deg;

        double yaw_gain;

        double desired_spin_deg;

        double lap_ang_vel_deg;

        size_t lap_start;

        bool enable_steering = true;

        bool enable_energy_feedback = true;

        double peak_phi_dot_threshold = 0;

        double EKp = 0;

        double EKi = 0;

        double EKd = 0;

        double tau_ff = 0;

        inline rnw_ros::RNWConfig to_config() const {
          rnw_ros::RNWConfig config;
          config.ang_vel_threshold = ang_vel_threshold;
          config.tau = tau;
          config.desired_nutation = desired_nutation;
          config.rocking_max_vel = rocking_max_vel;
          config.rocking_max_acc = rocking_max_acc;
          config.direct_control = direct_control;
          config.enable_energy_feedback = enable_energy_feedback;
          config.EKp = EKp;
          config.peak_phi_dot_threshold = peak_phi_dot_threshold;
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

    void load_from_ros( ros::NodeHandle & nh );

};


#endif //SRC_RNW_CONFIG_H
