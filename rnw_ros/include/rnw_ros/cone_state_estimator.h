//
// Created by sheep on 2020/9/8.
//

#ifndef SRC_CONE_STATE_ESTIMATOR_H
#define SRC_CONE_STATE_ESTIMATOR_H

#include "rnw_ros/rnw_utils.h"

struct cone_state_estimator_t {

    rnw_config_t rnw_config;

    Vector3d X_base_body() const;

    ros::Publisher pub_cone_state;

    ros::Publisher pub_odom_dt;

    ros::Subscriber sub_odom;

    nav_msgs::Odometry latest_odom;

    bool init = false;
    static constexpr double min_tilt = 5;
    double odom_timeout = 1;

    // latest states
    Vector3d latest_euler_angles;
    Vector3d latest_euler_velocity;
    Matrix3d R_markers;
    Vector3d T_markers;
    Vector3d T_tip;
    Vector3d T_base;
    Vector3d T_center;
    bool contact_valid = false;
    Vector3d contact_point;

    lpf_1st_butterworth_t lpf_ang_vel_x;
    lpf_1st_butterworth_t lpf_ang_vel_y;
    lpf_1st_butterworth_t lpf_ang_vel_z;

    explicit cone_state_estimator_t( ros::NodeHandle & nh );

    void on_odom( nav_msgs::OdometryConstPtr const & msg );

    void publish_cone_state( nav_msgs::OdometryConstPtr const & msg ) const;

    void update_euler_angles();

    void update_euler_velocity(nav_msgs::OdometryConstPtr const & msg);

    void update_body_points(nav_msgs::OdometryConstPtr const & msg);

    /**
     * 3D plane of object's base, A(x-x0)+B(y-y0)+C(z-z0)=0
     * where (A,B,C) is the normal vector, and (x0,y0,z0) is center of the disc
     * The plane intersect with z=ground_z, get a 2d line A'x+B'y+C'=0, normal vector (A',B')
     * (x0,y0) is disc center on z=ground_z plane
     * contact point = (x0,y0) + lambda * (A',B') which lies on A'x+B'y+C'=0
     */
    void update_contact_point();

};

#endif //SRC_CONE_STATE_ESTIMATOR_H
