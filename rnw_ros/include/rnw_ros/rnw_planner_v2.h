#ifndef RNW_ROS_RNW_PLANNER_V2_H
#define RNW_ROS_RNW_PLANNER_V2_H

#include "rnw_ros/rnw_utils.h"
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RnwCmd.h>
#include <rnw_msgs/RnwState.h>

/**
 * positive rot increase yaw
 * states are recorded before the step taken
 */
struct precession_regulator_t {

    explicit precession_regulator_t(rnw_config_t const & c );

    size_t step_count = 0;

    rnw_config_t const & rnw_config;

    double desired_heading;

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

    ros::Time stamp;

    uint32_t seq = 0;

    Vector3d setpoint;

    double heading;

};

struct rnw_planner_v2_t {

    explicit rnw_planner_v2_t( rnw_config_t const & config );

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg );

    rnw_msgs::RnwState to_rnw_state() const;

    /**
     * Main loop of the planner, run at 30Hz
     */
    void spin();

    rnw_command_t cmd;

    void start_walking();

    void stop_walking();

public:

    rnw_config_t const & rnw_config;

    ros::Publisher pub_rnw_state;

    bool cone_state_init = false;

    rnw_msgs::ConeState latest_cone_state;

    double rot_dir = -1;

    size_t step_count;

    bool is_walking = false;

    uint8_t walk_idx = 0;

    precession_regulator_t precession_regulator;

    void control_loop();

    void plan_cmd_walk();

};

#endif //RNW_ROS_RNW_PLANNER_V2_H
