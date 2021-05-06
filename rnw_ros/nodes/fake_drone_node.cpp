//
// Created by sheep on 2021/4/17.
//
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <n3ctrl/N3CtrlState.h>

#include "rnw_ros/ros_utils.h"

nav_msgs::Odometry cmd2odom( quadrotor_msgs::PositionCommand const & cmd ){
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = cmd.header.stamp;
  odom.pose.pose.position = cmd.position;
  odom.pose.pose.orientation.w = 1;
  return odom;
}

nav_msgs::Odometry pos2odom( double x, double y, double z ){
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation.w = 1;
  return odom;
}

struct fake_drone_t {

    ros::NodeHandle & nh;

    ros::Publisher pub_odom;

    ros::Publisher pub_n3ctrl;

    quadrotor_msgs::PositionCommand latest_position_cmd;

    nav_msgs::Odometry latest_odom;

    ros::Timer timer;

    ros::Subscriber sub_pos_cmd;

    explicit fake_drone_t( ros::NodeHandle & _nh ): nh(_nh) {

      latest_odom = pos2odom(
              get_param_default<double>(nh,"init_x",0),
              get_param_default<double>(nh,"init_y",0),
              get_param_default<double>(nh,"init_z",0)
      );

      pub_odom = nh.advertise<nav_msgs::Odometry>("odom",10);

      pub_n3ctrl = nh.advertise<n3ctrl::N3CtrlState>("n3ctrl",10);

      timer = nh.createTimer(ros::Rate(90),&fake_drone_t::on_timer,this);

      sub_pos_cmd = nh.subscribe<quadrotor_msgs::PositionCommand>(
              "position_cmd",
              10,
              &fake_drone_t::on_position_cmd,
              this,
              ros::TransportHints().tcpNoDelay()
      );

    }

    void on_position_cmd( quadrotor_msgs::PositionCommandConstPtr const & msg ){
      latest_position_cmd = *msg;
    }

    void on_timer( ros::TimerEvent const & event ){

      // execute position command

      if ( latest_position_cmd.header.stamp > latest_odom.header.stamp ) {
        latest_odom = cmd2odom(latest_position_cmd);
      }

      // publish odom

      latest_odom.header.stamp = ros::Time::now();
      pub_odom.publish(latest_odom);

      // publish n3ctrl_state

      constexpr double ctrl_timeout = 0.5;

      n3ctrl::N3CtrlState n3ctrl;
      n3ctrl.header.stamp = ros::Time::now();
      n3ctrl.header.frame_id = "world";
      n3ctrl.last_traj_id = 0;
      n3ctrl.state = (ros::Time::now() - latest_position_cmd.header.stamp).toSec() > ctrl_timeout ?
              n3ctrl::N3CtrlState::STATE_CMD_HOVER : n3ctrl::N3CtrlState::STATE_CMD_CTRL;
      pub_n3ctrl.publish(n3ctrl);

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"fake_drone_node");

  ros::NodeHandle nh("~");

  fake_drone_t fake_drone(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}