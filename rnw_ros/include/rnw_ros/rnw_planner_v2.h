#ifndef RNW_ROS_RNW_PLANNER_V2_H
#define RNW_ROS_RNW_PLANNER_V2_H

#include "rnw_ros/rnw_utils.h"
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RnwCmd.h>

/**
 * positive rot increase yaw
 * states are recorded before the step taken
 */
struct precession_regulator_t {

    explicit precession_regulator_t(rnw_config_t const & c );

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

};

struct rnw_command_t {

    uint32_t cmd_idx = 0;

    Vector3d control_point_setpoint;

    rnw_msgs::RnwCmd to_ros_msg() const;

};

struct rnw_planner_v2_t {

    rnw_planner_v2_t( ros::NodeHandle & nh, rnw_config_t const & config );

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg );

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
    rnw_command_t take_cmd();

    rnw_command_t rnw_command;

    void start_walking();

    void stop_walking();

private:

    enum class cmd_fsm_e {
        idle,
        pending,
        executing
    } cmd_fsm = cmd_fsm_e::idle;

    enum class cone_fsm_e {
        idle,
        qstatic,
        rocking
    } cone_fsm = cone_fsm_e::idle;

    void fsm_update();

    void fsm_transition( cone_fsm_e from, cone_fsm_e to );

    ros::NodeHandle & nh;

    rnw_config_t const & rnw_config;

    bool cone_state_init = false;

    rnw_msgs::ConeState latest_cone_state;

    double rot_dir = -1;

    size_t step_count;

    bool is_walking = false;

    uint8_t walk_idx = 0;

    double desired_yaw;

    precession_regulator_t precession_regulator;

    void plan_next_cmd();

    void plan_cmd_walk();

};

#endif //RNW_ROS_RNW_PLANNER_V2_H
