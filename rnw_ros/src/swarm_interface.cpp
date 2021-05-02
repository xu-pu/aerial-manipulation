#include "rnw_ros/swarm_interface.h"

quadrotor_msgs::PolynomialTrajectory swarm_interface_t::abort_traj() {
  quadrotor_msgs::PolynomialTrajectory msg;
  msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
  return msg;
}

swarm_interface_t::swarm_interface_t( ros::NodeHandle & _nh ) : nh(_nh) {

  pub_traj_drone1 = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/drone1/traj",10);

  pub_traj_drone2 = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/drone2/traj",10);

  sub_odom_drone1 = nh.subscribe<nav_msgs::Odometry>(
          "/drone1/odom",
          10,
          &swarm_interface_t::on_odom_drone1,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  sub_odom_drone2 = nh.subscribe<nav_msgs::Odometry>(
          "/drone2/odom",
          10,
          &swarm_interface_t::on_odom_drone2,
          this,
          ros::TransportHints().tcpNoDelay()
  );

}

bool swarm_interface_t::initialized() const {
  return init_drone1 && init_drone2;
}

void swarm_interface_t::on_odom_drone1( nav_msgs::OdometryConstPtr const & msg ){
  init_drone1 = true;
  latest_odom_drone1 = *msg;
}

void swarm_interface_t::on_odom_drone2( nav_msgs::OdometryConstPtr const & msg ){
  init_drone2 = true;
  latest_odom_drone2 = *msg;
}

void swarm_interface_t::send_traj( quadrotor_msgs::PolynomialTrajectory const & traj1, quadrotor_msgs::PolynomialTrajectory const & traj2 ) const {
  pub_traj_drone1.publish(traj1);
  pub_traj_drone2.publish(traj2);
}

void swarm_interface_t::send_traj_just_drone1( quadrotor_msgs::PolynomialTrajectory const & msg ) const {
  pub_traj_drone1.publish(msg);
  pub_traj_drone2.publish(abort_traj());
}

void swarm_interface_t::send_traj_just_drone2( quadrotor_msgs::PolynomialTrajectory const & msg ) const {
  pub_traj_drone1.publish(abort_traj());
  pub_traj_drone2.publish(msg);
}
