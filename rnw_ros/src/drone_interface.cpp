#include "rnw_ros/drone_interface.h"
#include "rnw_ros/rnw_utils.h"

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

void drone_interface_t::execute_trajectory(quadrotor_msgs::PolynomialTrajectory const & traj ) const {
  if ( ready(true) ) {
    pub_traj.publish(traj);
  }
}

void drone_interface_t::setup_trajectory_generator() {
  traj_generator = AmTraj(1024, 16, 0.4, max_vel, max_acc, 23, 0.02);
}

void drone_interface_t::set_max_vel(double val) {
  set_max_vel_acc(val,max_acc);
}

void drone_interface_t::set_max_acc(double val) {
  set_max_vel_acc(max_vel,val);
}

void drone_interface_t::set_max_vel_acc(double mvel, double macc) {
  max_vel = mvel;
  max_acc = macc;
  setup_trajectory_generator();
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::gen_traj_go_to_point( Vector3d const & to ){

  Vector3d from = uav_utils::from_point_msg(latest_odom.pose.pose.position);

  double dist = (to-from).norm();

  if ( dist < epsi_distance ) {
    return gen_setpoint_traj(latest_odom,to,0.5);
  }
  else {
    vector<Vector3d> waypoints;
    waypoints.emplace_back(from);
    waypoints.emplace_back((from+to)/2);
    waypoints.emplace_back(to);
    Vector3d v0 = Vector3d::Zero();
    auto traj = traj_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
    return to_ros_msg(traj,latest_odom,ros::Time::now());
  }

}

void drone_interface_t::go_to_point(const Vector3d &tgt) {

  if ( !ready(true) ) { return; }

  execute_trajectory(gen_traj_go_to_point(tgt));

}