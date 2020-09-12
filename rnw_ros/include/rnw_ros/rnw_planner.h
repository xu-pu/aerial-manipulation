//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_RNW_PLANNER_H
#define SRC_RNW_PLANNER_H

#include "rnw_ros/rnw_utils.h"
#include "rnw_msgs/ConeState.h"
#include "rnw_msgs/RockingCmd.h"

struct rnw_planner_t {

    ///////////////////////////////////////
    /// Public Interface
    ///////////////////////////////////////

    explicit rnw_planner_t( ros::NodeHandle & nh );

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg );

    void on_uav_odom( nav_msgs::OdometryConstPtr const & msg );

    /**
     * Main loop of the planner, run at 30Hz
     */
    void spin();

    /**
     * Call when the current command is executed
     * fsm: "executing" -> "idle"
     */
    void cmd_complete();

    /**
     * Take the current pending command
     * fsm: "pending" -> "executing"
     * @return rnw_cmd pointer
     */
    rnw_cmd_t * take_cmd();

    /**
     * All information output from the planner is in the rnw_cmd.
     * You only need to care about setpoint_uav, which is where the
     * UAV need to go. Other information are supplementary
     */
    rnw_cmd_t rnw_cmd;

    //////////////////////////////////////
    /// Interactive Triggers
    //////////////////////////////////////

    void start_walking();

    void stop_walking();

    void trigger_adjust_grip();

    void trigger_adjust_nutation();

    ///////////////////////////////
    /// FSM
    ///////////////////////////////

    enum class cone_fsm_e {
        idle, qstatic, rocking
    };

    cone_fsm_e fsm = cone_fsm_e::idle;

    void fsm_update();

    void fsm_transition( cone_fsm_e from, cone_fsm_e to );

    //////////////////////////
    /// ROS Stuff
    //////////////////////////

    rnw_config_t rnw_config;

    ros::Publisher pub_rocking_cmd;

    ros::Publisher pub_grip_state;

    bool cone_state_init = false;

    rnw_msgs::ConeState latest_cone_state;

    bool uav_odom_init = false;

    nav_msgs::Odometry latest_uav_odom;

    ////////////////////////////////////////////
    /// Planning

    static constexpr double deg2rad = M_PI/180.;

    static constexpr double min_tilt = 10 * deg2rad;

    double ang_vel_threshold = 0.5;

    double rot_dir = -1;

    double rot_amp_deg = 30;

    // rocking command

    bool request_adjust_grip = false;

    bool request_adjust_nutation = false;

    bool is_walking = false;

    void plan_cmd_walk();

    void plan_cmd_adjust_grip();

    void plan_cmd_adjust_nutation();

    // debug

    void on_debug_trigger( std_msgs::HeaderConstPtr const & msg );

};

#endif //SRC_RNW_PLANNER_H
