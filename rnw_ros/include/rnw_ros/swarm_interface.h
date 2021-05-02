//
// Created by sheep on 2021/5/2.
//

#ifndef RNW_ROS_SWARM_INTERFACE_H
#define RNW_ROS_SWARM_INTERFACE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>

struct swarm_interface_t {

    ros::NodeHandle & nh;

    ros::Publisher pub_traj_drone1;

    ros::Publisher pub_traj_drone2;

    ros::Subscriber sub_odom_drone1;

    ros::Subscriber sub_odom_drone2;

    bool init_drone1 = false;

    bool init_drone2 = false;

    nav_msgs::Odometry latest_odom_drone1;

    nav_msgs::Odometry latest_odom_drone2;

    static quadrotor_msgs::PolynomialTrajectory abort_traj();

    explicit swarm_interface_t( ros::NodeHandle & _nh );

    bool initialized() const;

    void on_odom_drone1( nav_msgs::OdometryConstPtr const & msg );

    void on_odom_drone2( nav_msgs::OdometryConstPtr const & msg );

    void send_traj( quadrotor_msgs::PolynomialTrajectory const & traj1, quadrotor_msgs::PolynomialTrajectory const & traj2 ) const;

    void send_traj_just_drone1( quadrotor_msgs::PolynomialTrajectory const & msg ) const;

    void send_traj_just_drone2( quadrotor_msgs::PolynomialTrajectory const & msg ) const;

};

#endif //RNW_ROS_SWARM_INTERFACE_H
