#include "rnw_ros/swarm_interface.h"

quadrotor_msgs::PolynomialTrajectory swarm_interface_t::abort_traj() {
  quadrotor_msgs::PolynomialTrajectory msg;
  msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
  return msg;
}

swarm_interface_t::swarm_interface_t( ros::NodeHandle & _nh ) : nh(_nh) {

  pub_abort = nh.advertise<std_msgs::Header>("/abort",10);

  pub_traj_drone1 = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/drone1/traj",10);

  pub_traj_drone2 = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/drone2/traj",10);

  sub_odom_drone1 = nh.subscribe<nav_msgs::Odometry>(
          "/drone1/odom",
          1,
          &swarm_interface_t::on_odom_drone1,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  sub_odom_drone2 = nh.subscribe<nav_msgs::Odometry>(
          "/drone2/odom",
          1,
          &swarm_interface_t::on_odom_drone2,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  sub_n3ctrl_drone1 = nh.subscribe<n3ctrl::N3CtrlState>(
          "/drone1/state",
          1,
          &swarm_interface_t::on_n3ctrl_drone1,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  sub_n3ctrl_drone2 = nh.subscribe<n3ctrl::N3CtrlState>(
          "/drone2/state",
          1,
          &swarm_interface_t::on_n3ctrl_drone2,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  integrity_check_timer = nh.createTimer(ros::Rate(30),&swarm_interface_t::on_integrity_check,this);

}

bool swarm_interface_t::ready() const {
  return drone1_connected() && drone2_connected() &&
          latest_n3ctrl_drone1.state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER &&
          latest_n3ctrl_drone2.state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER;
}

void swarm_interface_t::on_odom_drone1( nav_msgs::OdometryConstPtr const & msg ){
  latest_odom_drone1 = *msg;
}

void swarm_interface_t::on_odom_drone2( nav_msgs::OdometryConstPtr const & msg ){
  latest_odom_drone2 = *msg;
}

void swarm_interface_t::send_traj( quadrotor_msgs::PolynomialTrajectory const & traj1, quadrotor_msgs::PolynomialTrajectory const & traj2 ) const {
  if ( ready() ) {
    pub_traj_drone1.publish(traj1);
    pub_traj_drone2.publish(traj2);
  }
  else {
    ROS_WARN_STREAM("[swarm] swarm is not ready, can not send trajectories");
  }
}

void swarm_interface_t::send_traj_just_drone1( quadrotor_msgs::PolynomialTrajectory const & msg ) const {
  send_traj(msg,abort_traj());
}

void swarm_interface_t::send_traj_just_drone2( quadrotor_msgs::PolynomialTrajectory const & msg ) const {
  send_traj(abort_traj(),msg);
}

void swarm_interface_t::on_n3ctrl_drone1(const n3ctrl::N3CtrlStateConstPtr &msg) {
  latest_n3ctrl_drone1 = *msg;
}

void swarm_interface_t::on_n3ctrl_drone2(const n3ctrl::N3CtrlStateConstPtr &msg) {
  latest_n3ctrl_drone2 = *msg;
}

void swarm_interface_t::on_integrity_check(const ros::TimerEvent &e) {

  bool d1_connected = drone1_connected();
  bool d2_connected = drone2_connected();

  if ( drone1_connected_latch && !d1_connected ) {
    ROS_ERROR_STREAM("[swarm] drone1 disconnected!");
  }
  else if ( !drone1_connected_latch && d1_connected ) {
    ROS_INFO_STREAM("[swarm] drone1 connected");
  }

  if ( drone2_connected_latch && !d2_connected ) {
    ROS_ERROR_STREAM("[swarm] drone2 disconnected!");
  }
  else if ( !drone2_connected_latch && d2_connected ) {
    ROS_INFO_STREAM("[swarm] drone2 connected");
  }

  drone1_connected_latch = d1_connected;
  drone2_connected_latch = d2_connected;

  bool swarm_ready = ready();
  if ( swarm_ready_latch && !swarm_ready ) {
    on_exit_ready();
  }
  else if ( !swarm_ready_latch && swarm_ready ) {
    on_became_ready();
  }

  swarm_ready_latch = swarm_ready;

}

bool swarm_interface_t::drone1_connected() const {
  return (ros::Time::now() - latest_odom_drone1.header.stamp).toSec() < odom_timeout_sec &&
          (ros::Time::now() - latest_n3ctrl_drone1.header.stamp).toSec() < n3ctrl_timeout_sec;
}

bool swarm_interface_t::drone2_connected() const {
  return (ros::Time::now() - latest_odom_drone2.header.stamp).toSec() < odom_timeout_sec &&
         (ros::Time::now() - latest_n3ctrl_drone2.header.stamp).toSec() < n3ctrl_timeout_sec;
}

void swarm_interface_t::on_became_ready() const {
  ROS_INFO_STREAM("[swarm] bacame ready");
}

void swarm_interface_t::on_exit_ready() const {
  ROS_ERROR_STREAM("[swarm] exit ready state! abort!");
  send_abort();
}

void swarm_interface_t::send_abort() const {
  std_msgs::Header m;
  m.stamp = ros::Time::now();
  pub_abort.publish(m);
}