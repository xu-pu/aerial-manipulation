//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_RNW_CONFIG_H
#define SRC_RNW_CONFIG_H

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/RNWConfig.h"

struct rnw_config_t {

    Vector3d X_tcp_cage;

    Vector3d flu_T_tcp;

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


#endif //SRC_RNW_CONFIG_H
