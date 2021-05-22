//
// Created by sheep on 2021/5/21.
//

#ifndef RNW_ROS_DRONE_INTERFACE_H
#define RNW_ROS_DRONE_INTERFACE_H

#include <n3ctrl/N3CtrlState.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <am_traj/am_traj.hpp>
#include <am_traj/ros_msgs.h>

#include "rnw_ros/ros_utils.h"

struct drone_interface_t {

    static constexpr double message_timeout = 1;

    static constexpr double epsi_distance = 0.05;

    bool initialized = false;

    string name;

    nav_msgs::Odometry latest_odom;

    n3ctrl::N3CtrlState latest_n3ctrl;

    quadrotor_msgs::PolynomialTrajectory latest_traj;

    drone_interface_t();

    explicit drone_interface_t( string const & drone_name );

    void init( string const & drone_name );

    void on_odom( nav_msgs::OdometryConstPtr const & msg );

    void on_n3ctrl( n3ctrl::N3CtrlStateConstPtr const & msg );

    bool ready( bool print_reason = true ) const;

    bool odom_in_time() const;

    void execute_trajectory(quadrotor_msgs::PolynomialTrajectory const & traj ) const;

    void set_max_vel( double val );

    void set_max_acc( double val );

    void set_max_vel_acc( double mvel, double macc );

    quadrotor_msgs::PolynomialTrajectory gen_traj_go_to_point( Vector3d const & tgt ) const;

    quadrotor_msgs::PolynomialTrajectory gen_traj_go_to_point( Vector3d const & tgt, double yaw ) const;

    quadrotor_msgs::PolynomialTrajectory gen_traj_from_waypoint( vector<Vector3d> const & waypoints_in ) const;

    quadrotor_msgs::PolynomialTrajectory gen_traj_from_waypoint( vector<Vector3d> const & waypoints_in, double final_yaw ) const;

    void go_to_point( Vector3d const & target );

    void go_to_point_in_intermediate_frame( Vector3d const & point );

    void follow_waypoints( vector<Vector3d> const & waypoints );

    void follow_waypoints( vector<Vector3d> const & waypoints, double yaw );

    void follow_waypoints_in_intermediate_frame( vector<Vector3d> const & waypoints );

    AmTraj traj_generator = AmTraj(1024, 16, 0.4, 0.5, 0.5, 23, 0.02);

private:

    void setup_trajectory_generator();

    double max_vel = 1;

    double max_acc = 1;

    ros::Subscriber sub_odom;

    ros::Subscriber sub_n3ctrl;

    ros::Publisher pub_traj;

};

#endif //RNW_ROS_DRONE_INTERFACE_H
