//
// Created by sheep on 2021/5/22.
//

#ifndef RNW_ROS_CAGING_STATE_H
#define RNW_ROS_CAGING_STATE_H

#include "rnw_ros/rnw_utils.h"

struct caging_state_t {

    caging_state_t();

    /**
     * Calculate the current grip state
     * @param cone_state
     * @param uav_odom
     * @param flu_T_tcp
     */
    caging_state_t(
            rnw_msgs::ConeState const & cone_state,
            nav_msgs::Odometry const & uav_odom,
            Vector3d const & flu_T_tcp
    );

    rnw_msgs::ConeState cone_state;

    nav_msgs::Odometry uav_odom;

    Vector3d flu_T_tcp;

    Vector3d grip_point;

    bool grip_valid = false;

    double grip_radius = 0;

    double grip_depth = 0;

    bool initialized = false;

    rnw_msgs::GripState to_msg() const;

};

#endif //RNW_ROS_CAGING_STATE_H
