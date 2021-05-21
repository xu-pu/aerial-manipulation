//
// Created by sheep on 2021/5/21.
//

#ifndef RNW_ROS_CONE_INTERFACE_H
#define RNW_ROS_CONE_INTERFACE_H

#include <ros/ros.h>
#include <rnw_msgs/ConeState.h>

struct cone_interface_t {

    static constexpr double msg_timeout = 1;

    cone_interface_t();

    rnw_msgs::ConeState latest_cone_state;

    ros::Subscriber sub_cone_state;

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg );

};

#endif //RNW_ROS_CONE_INTERFACE_H
