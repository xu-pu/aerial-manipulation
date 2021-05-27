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
  ROS_WARN("[%s] reconfigure max_vel=%f, max_acc=%f",name.c_str(),mvel,macc);
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::gen_traj_go_to_point( Vector3d const & to ) const {
  return gen_traj_go_to_point(to,uav_yaw_from_odom(latest_odom));
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::gen_traj_go_to_point( Vector3d const & to, double yaw ) const {

  Vector3d from = uav_utils::from_point_msg(latest_odom.pose.pose.position);

  double dist = (to-from).norm();

  if ( dist < epsi_distance ) {
    return gen_setpoint_traj(latest_odom,to,yaw,0.5);
  }
  else {
    vector<Vector3d> waypoints;
    waypoints.emplace_back(from);
    waypoints.emplace_back((from+to)/2);
    waypoints.emplace_back(to);
    Vector3d v0 = Vector3d::Zero();
    auto traj = traj_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
    return to_ros_msg(traj,uav_yaw_from_odom(latest_odom),yaw,ros::Time::now());
  }

}

void drone_interface_t::go_to_point( const Vector3d & target) const {

  if (!ready()) return;

  execute_trajectory(gen_traj_go_to_point(target));

}

void drone_interface_t::go_to_point_in_intermediate_frame( const Vector3d & point ) const {

  if (!ready()) return;

  Vector3d target = point_in_intermediate_frame(point,latest_odom);

  execute_trajectory(gen_traj_go_to_point(target));

}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::gen_traj_from_waypoint( const vector<Vector3d> & waypoints_in ) const {
  return gen_traj_from_waypoint(waypoints_in,uav_yaw_from_odom(latest_odom));
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::gen_traj_from_waypoint( vector<Vector3d> const & waypoints_in, double final_yaw ) const {

  vector<Vector3d> waypoints_out;
  // make sure always start with the drone's current position
  waypoints_out.emplace_back(uav_utils::from_point_msg(latest_odom.pose.pose.position));

  for ( Vector3d const & iter : waypoints_in ) {
    if ((waypoints_out.back() - iter).norm() > epsi_distance ) {
      waypoints_out.emplace_back(iter);
    }
  }

  // always end with the last point in the original waypoints
  waypoints_out.back() = waypoints_in.back();

  if (waypoints_out.size() < 3 ) {
    return gen_traj_go_to_point(waypoints_out.back());
  }

  Vector3d v0 = Vector3d::Zero();
  auto traj = traj_generator.genOptimalTrajDTC(waypoints_out, v0, v0, v0, v0);
  return to_ros_msg(traj, uav_yaw_from_odom(latest_odom), final_yaw, ros::Time::now());

}

void drone_interface_t::follow_waypoints(const vector<Vector3d> &waypoints) const {

  if (!ready()) return;

  execute_trajectory(gen_traj_from_waypoint(waypoints));

}

void drone_interface_t::follow_waypoints( vector<Vector3d> const & waypoints, double yaw ) const {

  if (!ready()) return;

  execute_trajectory(gen_traj_from_waypoint(waypoints,yaw));

}

void drone_interface_t::follow_waypoints_in_intermediate_frame(const vector<Vector3d> & points) const {

  if (!ready()) return;

  vector<Vector3d> waypoints = points_in_intermediate_frame(points,latest_odom);

  execute_trajectory(gen_traj_from_waypoint(waypoints));

}

bool drone_interface_t::odom_in_time() const {
  return message_in_time(latest_odom,message_timeout);
}