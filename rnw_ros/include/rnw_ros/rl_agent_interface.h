//
// Created by sheep on 2021/8/30.
//

#ifndef RNW_ROS_RL_AGENT_INTERFACE_H
#define RNW_ROS_RL_AGENT_INTERFACE_H

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/cone_interface.h"
#include "rnw_ros/rnw_config.h"

#include <sensor_msgs/Joy.h>

struct rl_agent_interface_t {

    rnw_config_t & config;

    static constexpr size_t obs_rate = 10;

    Vector3d latest_cmd;

    sensor_msgs::Joy latest_action;

    sensor_msgs::Joy latest_obs;

    cone_interface_t cone;

    ros::Subscriber sub_action;

    ros::Publisher pub_obs;

    ros::Publisher pub_cmd_vel;

    ros::Timer timer_obs;

    explicit rl_agent_interface_t( rnw_config_t & );

    void on_action( sensor_msgs::JoyConstPtr const & );

    void on_observation( ros::TimerEvent const & e );

};

#endif //RNW_ROS_RL_AGENT_INTERFACE_H
