#ifndef RNW_ROS_RNW_PLANNER_V2_H
#define RNW_ROS_RNW_PLANNER_V2_H

#include "rnw_ros/rnw_utils.h"
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RnwCmd.h>
#include <rnw_msgs/RnwState.h>

/**
 * Information required to carry out the tilting
 * no more, no less
 */
struct rnw_command_t {

    ros::Time stamp;

    uint32_t seq = 0;

    /**
     * Desired position of the control point
     */
    Vector3d setpoint;

    /**
     * Heading direction of r-n-w
     * This is used to adjust yaw of robots
     */
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

    void start_walking();

    void stop_walking();

public:

    rnw_config_t const & rnw_config;

    ros::Publisher pub_rnw_state;

    rnw_msgs::ConeState latest_cone_state;

    bool cone_state_init = false;

    bool is_walking = false;

    size_t walk_idx = 0;

    size_t step_count = 0;

    double step_direction = -1;

    /**
     * Indicates is r-n-w initialized
     * before and after initialization has different parameters
     */

    bool energy_initialized = false;

    /**
     * Current rocking magnitude, updated upon each positive step
     */

    double latest_tau_rad = 0;

    /**
     * Here we store the heading direction at the latest positive and negative step
     */

    double heading_step_pos = 0;

    double heading_step_neg = 0;

    double desired_heading_direction = 0;

    /**
     * Maximum phi_dot since last step, it represents the current energy level
     */

    double peak_phi_dot = 0;

    vector<double> peak_phi_dot_history;

    double energy_err_integral = 0;

    rnw_command_t cmd;

    void control_loop();

    void plan_cmd_walk();

};

#endif //RNW_ROS_RNW_PLANNER_V2_H
