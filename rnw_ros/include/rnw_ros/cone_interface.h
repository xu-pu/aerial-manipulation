//
// Created by sheep on 2021/5/21.
//

#ifndef RNW_ROS_CONE_INTERFACE_H
#define RNW_ROS_CONE_INTERFACE_H

#include <ros/ros.h>
#include <rnw_msgs/ConeState.h>

#include "rnw_ros/ros_utils.h"

struct cone_interface_t {

    static constexpr double msg_timeout = 1;

    cone_interface_t();

    rnw_msgs::ConeState latest_cone_state;

    ros::Subscriber sub_cone_state;

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg );

    bool odom_in_time() const;

public:

    Vector3d contact_point() const;

    Vector3d tip() const;

    Vector3d tip_at_rest() const;

    Vector3d tip_at_nutation( double rad ) const;

    Matrix3d rnw_frame() const;

};

#endif //RNW_ROS_CONE_INTERFACE_H
