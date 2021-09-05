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

    enum trigger_e {
        trigger_idle = 0,
        trigger_arm_motors = 1,
    };

    static constexpr double message_timeout = 1;

    static constexpr double epsi_distance = 0.05;

    bool initialized = false;

    string name;

    double cable_length;

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

    Vector3d position() const;

    double yaw() const;

    void set_max_vel( double val );

    void set_max_acc( double val );

    void set_max_vel_acc( double mvel, double macc );

public: // these method only perform planning

    quadrotor_msgs::PolynomialTrajectory plan( AmTraj const & generator, vector<Vector3d> const & waypoints_in, double final_yaw ) const;

    quadrotor_msgs::PolynomialTrajectory plan( AmTraj const & generator, Vector3d const & tgt, double yaw ) const;

    quadrotor_msgs::PolynomialTrajectory plan( AmTraj const & generator, vector<Vector3d> const & waypoints_in ) const;

    quadrotor_msgs::PolynomialTrajectory plan( AmTraj const & generator, Vector3d const & tgt ) const;

    quadrotor_msgs::PolynomialTrajectory plan( Vector3d const & tgt ) const;

    quadrotor_msgs::PolynomialTrajectory plan( Vector3d const & tgt, double yaw ) const;

    quadrotor_msgs::PolynomialTrajectory plan( vector<Vector3d> const & waypoints_in ) const;

    quadrotor_msgs::PolynomialTrajectory plan( vector<Vector3d> const & waypoints_in, double final_yaw ) const;

public: // these methods will take ACTION

    void arm_motors() const;

    void reset_traj() const;

    void cmd_pos_vel( Vector3d const & pos, Vector3d const & vel ) const;

    void cmd_pos_vel_yaw( Vector3d const & pos, Vector3d const & vel, double yaw ) const;

    void execute_trajectory(quadrotor_msgs::PolynomialTrajectory const & traj, bool do_not_check = false ) const;

    void go_to_point( Vector3d const & target ) const;

    void go_to_point_in_intermediate_frame( Vector3d const & point ) const;

    void follow_waypoints( vector<Vector3d> const & waypoints ) const;

    void follow_waypoints( vector<Vector3d> const & waypoints, double yaw ) const;

    void follow_waypoints_in_intermediate_frame( vector<Vector3d> const & waypoints ) const;

public: // static helpers

    static AmTraj create_setting( double max_vel, double max_acc );

    static bool n3ctrl_accept_traj( n3ctrl::N3CtrlState const & );

private:

    void setup_trajectory_generator();

    AmTraj traj_generator = AmTraj(1024, 16, 0.4, 0.5, 0.5, 23, 0.02);

    double max_vel = 1;

    double max_acc = 1;

    ros::Subscriber sub_odom;

    ros::Subscriber sub_n3ctrl;

    ros::Publisher pub_traj;

    ros::Publisher pub_pos_vel_cmd;

    ros::Publisher pub_trigger;

    bool just_checking = false;

};

#endif //RNW_ROS_DRONE_INTERFACE_H
