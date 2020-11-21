//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_RNW_PLANNER_H
#define SRC_RNW_PLANNER_H

#include "rnw_ros/rnw_utils.h"
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RockingCmd.h>
#include <rnw_msgs/WalkingState.h>

struct rnw_cmd_t {

    rnw_msgs::RockingCmd to_msg() const;

    //////////////////////////////////////
    /// Planner State
    //////////////////////////////////////

    enum cmd_fsm_e {
        fsm_idle,
        fsm_pending,
        fsm_executing
    } fsm = fsm_idle;

    enum cmd_type_e {
        cmd_rocking,
        cmd_adjust_grip,
        cmd_adjust_nutation
    } cmd_type = cmd_rocking;

    size_t cmd_idx = 0; // same cmd may be published multiple times, use this to keep track

    //////////////////////////////////////
    /// Waypoint
    //////////////////////////////////////

    double desired_yaw;

    Vector3d setpoint_uav;

    Vector3d setpoint_apex;

    //////////////////////////////////////
    /// Grip Monitoring
    //////////////////////////////////////

    grip_state_t grip_state;

    double setpoint_nutation;

    double setpoint_grip_depth;

    double err_grip_depth;

    double err_nutation_deg;

    //////////////////////////////////////
    /// Walking
    //////////////////////////////////////

    double tau_deg;

    size_t step_count;

    bool is_walking = false;

    uint8_t walk_idx = 0;

};

/**
 * positive rot increase yaw
 * states are recorded before the step taken
 */
struct steering_controller_t {

    explicit steering_controller_t(rnw_config_t const & c );

    size_t step_count = 0;

    rnw_config_t const & rnw_config;

    double desired_heading_yaw;

    double cur_relative_yaw;

    rnw_msgs::ConeState last_step;

    rnw_msgs::ConeState last_step_odd;

    rnw_msgs::ConeState last_step_even;

    /**
     * call before step taken
     * record latest odd and even step,
     * update step counter,
     * update heading direction
     */
    void step( rnw_msgs::ConeState const & cone_state );

    void start( rnw_msgs::ConeState const & cone_state );

    void end();

    /**
     * call after start()
     * @return
     */
    double desired_uav_yaw() const;

};

struct corridor_controller_t {

    rnw_config_t const & config;

    bool _init = false;

    Vector3d corridor_origin;

    double corridor_dir = 0; // yaw angle of the walking direction

    double corridor_width = 0; // tau

    // T_corridor_world
    Eigen::Matrix2d Rcw;
    Eigen::Vector2d Tcw;

    // T_world_corridor
    Eigen::Matrix2d Rwc;
    Eigen::Vector2d Twc;

    explicit corridor_controller_t( rnw_config_t const & cfg );

    /**
     * setup the corridor, origin+dir+width
     * @param cone_state
     */
    void init( rnw_msgs::ConeState const & cone_state );

    /**
     * update corridor width based on energy
     * @param cone_state
     */
    void update_cone_state( rnw_msgs::ConeState const & cone_state );

    /**
     * Calculate the next C position
     * @param cone_state
     * @param dir - >0 := left, <0 := right
     * @return
     */
    Vector3d calc_next_c( rnw_msgs::ConeState const & cone_state, int dir );

};

/**
 * E is energy, we use E := abs(phi), phi_dot == 0
 */
struct energy_feedback_t {

    size_t step_count = 0;

    bool _init = false;

    void init( rnw_msgs::ConeState const & cone_state );

    void step( rnw_msgs::ConeState const & cone_state );

    double E;
    double E_dot;
    double E_dot_dot;

};

struct rnw_planner_t {

    ///////////////////////////////////////
    /// Public Interface
    ///////////////////////////////////////

    rnw_planner_t( ros::NodeHandle & nh, rnw_config_t const & config );

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

    void trigger_adjust_yaw();

    void on_debug_trigger( std_msgs::HeaderConstPtr const & msg );

private:

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

    ros::NodeHandle & nh;

    rnw_config_t const & rnw_config;

    ros::Publisher pub_walking_state;

    ros::Publisher pub_rocking_cmd;

    ros::Publisher pub_grip_state;

    bool cone_state_init = false;

    rnw_msgs::ConeState latest_cone_state;

    bool uav_odom_init = false;

    nav_msgs::Odometry latest_uav_odom;

    ////////////////////////////////////////////
    /// Planning

    double rot_dir = -1;

    double integration_term = 0;

    // rocking command

    bool request_adjust_grip = false;

    bool request_adjust_nutation = false;

    steering_controller_t walking_state;

    energy_feedback_t energy_feedback;

    corridor_controller_t corridor;

    void plan_next_cmd();

    void plan_cmd_walk();

    void plan_cmd_walk_corridor();

    void plan_cmd_walk_no_feedforward();

    void plan_cmd_adjust_grip();

    void plan_cmd_adjust_nutation();

};

#endif //SRC_RNW_PLANNER_H
