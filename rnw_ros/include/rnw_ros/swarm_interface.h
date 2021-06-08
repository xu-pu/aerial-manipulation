//
// Created by sheep on 2021/5/2.
//

#ifndef RNW_ROS_SWARM_INTERFACE_H
#define RNW_ROS_SWARM_INTERFACE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <n3ctrl/N3CtrlState.h>

#include "rnw_ros/drone_interface.h"

struct swarm_interface_t {

    drone_interface_t drone1;

    drone_interface_t drone2;

    ros::NodeHandle & nh;

    ros::Publisher pub_abort;

    static quadrotor_msgs::PolynomialTrajectory abort_traj();

    explicit swarm_interface_t( ros::NodeHandle & _nh );

    void on_became_ready() const;

    void on_exit_ready() const;

    void on_integrity_check( ros::TimerEvent const & e );

    bool ready( bool print_reason = true ) const;

    void send_abort() const;

public: // these method will TAKE ACTION

    void send_traj( quadrotor_msgs::PolynomialTrajectory const & traj1, quadrotor_msgs::PolynomialTrajectory const & traj2 ) const;

    void send_traj_just_drone1( quadrotor_msgs::PolynomialTrajectory const & msg ) const;

    void send_traj_just_drone2( quadrotor_msgs::PolynomialTrajectory const & msg ) const;

    void go_to( Vector3d const & pt1, Vector3d const & pt2 ) const;

    void go_to( Vector3d const & pt1, Vector3d const & pt2, AmTraj const & setting ) const;

    void follow( vector<Vector3d> const & waypoints1, vector<Vector3d> const & waypoints2 ) const;

    void follow( vector<Vector3d> const & waypoints1, vector<Vector3d> const & waypoints2, AmTraj const & setting ) const;

    void arm_motors() const;

private:

    ros::Timer integrity_check_timer;

    bool swarm_ready_latch = false;

    bool check_swarm_ready = true;

};

#endif //RNW_ROS_SWARM_INTERFACE_H
