//
// Created by sheep on 2021/5/2.
//

#ifndef RNW_ROS_SWARM_INTERFACE_H
#define RNW_ROS_SWARM_INTERFACE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <n3ctrl/N3CtrlState.h>

struct swarm_interface_t {

    ros::NodeHandle & nh;

    ros::Publisher pub_abort;

    ros::Publisher pub_traj_drone1;

    ros::Publisher pub_traj_drone2;

    ros::Subscriber sub_odom_drone1;

    ros::Subscriber sub_odom_drone2;

    ros::Subscriber sub_n3ctrl_drone1;

    ros::Subscriber sub_n3ctrl_drone2;

    nav_msgs::Odometry latest_odom_drone1;

    nav_msgs::Odometry latest_odom_drone2;

    n3ctrl::N3CtrlState latest_n3ctrl_drone1;

    n3ctrl::N3CtrlState latest_n3ctrl_drone2;

    static quadrotor_msgs::PolynomialTrajectory abort_traj();

    explicit swarm_interface_t( ros::NodeHandle & _nh );

    void on_odom_drone1( nav_msgs::OdometryConstPtr const & msg );

    void on_odom_drone2( nav_msgs::OdometryConstPtr const & msg );

    void on_n3ctrl_drone1( n3ctrl::N3CtrlStateConstPtr const & msg );

    void on_n3ctrl_drone2( n3ctrl::N3CtrlStateConstPtr const & msg );

    void on_became_ready() const;

    void on_exit_ready() const;

    void send_traj( quadrotor_msgs::PolynomialTrajectory const & traj1, quadrotor_msgs::PolynomialTrajectory const & traj2 ) const;

    void send_traj_just_drone1( quadrotor_msgs::PolynomialTrajectory const & msg ) const;

    void send_traj_just_drone2( quadrotor_msgs::PolynomialTrajectory const & msg ) const;

    void on_integrity_check( ros::TimerEvent const & e );

    bool drone1_connected() const;

    bool drone2_connected() const;

    bool ready() const;

    void send_abort() const;

private:

    ros::Timer integrity_check_timer;

    bool drone1_connected_latch = false;

    bool drone2_connected_latch = false;

    bool swarm_ready_latch = false;

    static constexpr double odom_timeout_sec = 0.5;

    static constexpr double n3ctrl_timeout_sec = 1;

};

#endif //RNW_ROS_SWARM_INTERFACE_H
