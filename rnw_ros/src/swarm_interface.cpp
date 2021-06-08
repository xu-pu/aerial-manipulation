#include "rnw_ros/swarm_interface.h"
#include "rnw_ros/ros_utils.h"

quadrotor_msgs::PolynomialTrajectory swarm_interface_t::abort_traj() {
  quadrotor_msgs::PolynomialTrajectory msg;
  msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
  return msg;
}

swarm_interface_t::swarm_interface_t( ros::NodeHandle & _nh ) : nh(_nh), drone1("drone1"), drone2("drone2") {

  check_swarm_ready = get_param_default<bool>(nh,"check_swarm_ready",true);

  pub_abort = nh.advertise<std_msgs::Header>("/abort",10);

  integrity_check_timer = nh.createTimer(ros::Rate(30),&swarm_interface_t::on_integrity_check,this);

}

bool swarm_interface_t::ready( bool print_reason ) const {
  if ( check_swarm_ready ) {
    return drone1.ready(print_reason) && drone2.ready(print_reason);
  }
  else {
    return true;
  }
}

void swarm_interface_t::send_traj( quadrotor_msgs::PolynomialTrajectory const & traj1, quadrotor_msgs::PolynomialTrajectory const & traj2 ) const {
  if (!check_swarm_ready) { ROS_WARN_STREAM("[swarm] did not check swarm readiness, debug only, don't actually fly!"); }
  if ( ready() ) {
    drone1.execute_trajectory(traj1);
    drone2.execute_trajectory(traj2);
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

void swarm_interface_t::on_integrity_check(const ros::TimerEvent &e) {

  bool swarm_ready = ready(false);
  if ( swarm_ready_latch && !swarm_ready ) {
    on_exit_ready();
  }
  else if ( !swarm_ready_latch && swarm_ready ) {
    on_became_ready();
  }

  swarm_ready_latch = swarm_ready;

}

void swarm_interface_t::on_became_ready() const {
  ROS_INFO_STREAM("[swarm] bacame ready");
}

void swarm_interface_t::on_exit_ready() const {
  ROS_ERROR_STREAM("[swarm] exit ready state! abort!");
  ready(true);
  send_abort();
}

void swarm_interface_t::send_abort() const {
  std_msgs::Header m;
  m.stamp = ros::Time::now();
  pub_abort.publish(m);
}

void swarm_interface_t::go_to(const Vector3d &pt1, const Vector3d &pt2) const {
  send_traj(drone1.plan(pt1),drone2.plan(pt2));
}

void swarm_interface_t::follow(const vector<Vector3d> &waypoints1, const vector<Vector3d> &waypoints2) const {
  send_traj(drone1.plan((waypoints1)),drone2.plan(waypoints2));
}

void swarm_interface_t::go_to( Vector3d const & pt1, Vector3d const & pt2, AmTraj const & setting ) const {
  send_traj(drone1.plan(setting,pt1),drone2.plan(setting,pt2));
}

void swarm_interface_t::follow( vector<Vector3d> const & waypoints1, vector<Vector3d> const & waypoints2, AmTraj const & setting ) const {
  send_traj(drone1.plan(setting,waypoints1),drone2.plan(setting,waypoints2));
}

void swarm_interface_t::arm_motors() const {
  drone1.arm_motors();
  drone2.arm_motors();
}