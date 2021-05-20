#include "rnw_ros/drone_interface.h"

drone_interface_t::drone_interface_t() = default;

drone_interface_t::drone_interface_t( string const & drone_name ){
  init(drone_name);
}

void drone_interface_t::init( string const & drone_name ){

  ros::NodeHandle nh("~");

  stringstream topic_odom; topic_odom << "/" << drone_name << "/odom";

  sub_odom = nh.subscribe<nav_msgs::Odometry>(
          topic_odom.str(),
          1,
          &drone_interface_t::on_odom,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  stringstream topic_n3ctrl; topic_n3ctrl << "/" << drone_name << "/state";

  sub_n3ctrl = nh.subscribe<n3ctrl::N3CtrlState>(
          topic_n3ctrl.str(),
          1,
          &drone_interface_t::on_n3ctrl,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  stringstream topic_traj; topic_traj << "/" << drone_name << "/traj";

  pub_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>(topic_traj.str(),10);

  name = drone_name;

  initialized = true;

}

void drone_interface_t::on_odom( nav_msgs::OdometryConstPtr const & msg ){
  latest_odom = *msg;
}

void drone_interface_t::on_n3ctrl( n3ctrl::N3CtrlStateConstPtr const & msg ){
  latest_n3ctrl = *msg;
}

bool drone_interface_t::ready( bool print_reason ) const {

  if ( !initialized ) {
    if (print_reason) ROS_ERROR_STREAM("[drone_interface] ros interface not initialized");
    return false;
  }

  if ( !message_in_time(latest_odom,message_timeout) ) {
    if (print_reason) ROS_ERROR_STREAM("[drone_interface] odom timeout!");
    return false;
  }

  if ( !message_in_time(latest_n3ctrl,message_timeout) ) {
    if (print_reason) ROS_ERROR_STREAM("[drone_interface] n3ctrl timeout!");
    return false;
  }

  if ( latest_n3ctrl.state < n3ctrl::N3CtrlState::STATE_CMD_HOVER ) {
    if (print_reason) ROS_ERROR_STREAM("[drone_interface] n3ctrl won't take cmd at current state!");
    return false;
  }

  return true;

}

void drone_interface_t::send_traj( quadrotor_msgs::PolynomialTrajectory const & traj ) const {
  if ( ready(true) ) {
    pub_traj.publish(traj);
  }
}