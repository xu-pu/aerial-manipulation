//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_RNW_PLANNER_H
#define SRC_RNW_PLANNER_H

#include "rnw_ros/rnw_utils.h"

struct rnw_planner_t {

    // interface to other modules

    void start_planning_cmd();

    void stop_planning_cmd();

    /**
     * Call when the current command is complete
     */
    void cmd_ack();

    /**
     * Check is there command to execute
     */
    bool has_pending_cmd() const;

    Vector3d next_position() const;

    /**
     * Call on every new ConeState message
     * @param msg
     */
    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg );

    ///////////////////////////////

    static constexpr double deg2rad = M_PI/180.;

    static constexpr double min_tilt = 10 * deg2rad;

    double ang_vel_threshold = 0.5;

    enum class cone_fsm_e {
        idle, qstatic, rocking
    };

    ros::Publisher pub_rocking_cmd;

    cone_fsm_e fsm;

    rnw_msgs::ConeState latest_cone_state;

    // planning

    double rot_dir = -1;

    double rot_amp_deg = 30;

    size_t step_count = 0;

    // rocking command

    bool plan_cmd = false;

    rnw_msgs::RockingCmd latest_cmd;

    bool cmd_pending = false;

    size_t cmd_idx = 0;

    explicit rnw_planner_t( ros::NodeHandle & nh );

    void plan_next_position();

    void update_state( rnw_msgs::ConeState const & msg );

    void state_transition( cone_fsm_e from, cone_fsm_e to );

    // debug

    void on_debug_trigger( std_msgs::HeaderConstPtr const & msg );

};

#endif //SRC_RNW_PLANNER_H
