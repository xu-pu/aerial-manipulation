#include "rnw_ros/drone_interface.h"
#include "rnw_ros/rnw_utils.h"

#include <std_msgs/UInt8.h>
#include <quadrotor_msgs/PositionCommand.h>

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

  stringstream topic_trigger; topic_trigger << "/" << drone_name << "/trigger";

  pub_trigger = nh.advertise<std_msgs::UInt8>(topic_trigger.str(),100);

  stringstream topic_pos_vel_cmd; topic_pos_vel_cmd << "/" << drone_name << "/position_cmd";

  pub_pos_vel_cmd = nh.advertise<quadrotor_msgs::PositionCommand>(topic_pos_vel_cmd.str(),10);

  name = drone_name;

  stringstream param_cable; param_cable << "/cable/" << name;
  cable_length = get_ros_param_required<double>(nh,param_cable.str());

  initialized = true;

  just_checking = get_param_default<bool>(nh,"/just_checking",false);

}

void drone_interface_t::on_odom( nav_msgs::OdometryConstPtr const & msg ){
  latest_odom = *msg;
}

void drone_interface_t::on_n3ctrl( n3ctrl::N3CtrlStateConstPtr const & msg ){
  if ( n3ctrl_accept_traj(latest_n3ctrl) != n3ctrl_accept_traj(*msg) ) {
    // if drone's availability changed, make sure current traj is discarded
    reset_traj();
  }
  latest_n3ctrl = *msg;
}

bool drone_interface_t::ready( bool print_reason ) const {

  if ( just_checking ) {
    ROS_ERROR("[%s] not actually checking the system, for testing only!",name.c_str());
    return true;
  }

  if ( !initialized ) {
    if (print_reason) ROS_ERROR("[%s] ros interface not initialized",name.c_str());
    return false;
  }

  if ( !message_in_time(latest_odom,message_timeout) ) {
    if (print_reason) ROS_ERROR("[%s] odom timeout!",name.c_str());
    return false;
  }

  if ( !message_in_time(latest_n3ctrl,message_timeout) ) {
    if (print_reason) ROS_ERROR("[%s] n3ctrl timeout!",name.c_str());
    return false;
  }

  if ( latest_n3ctrl.state < n3ctrl::N3CtrlState::STATE_CMD_HOVER ) {
    if (print_reason) ROS_ERROR("[%s] n3ctrl won't take cmd at current state!",name.c_str());
    return false;
  }

  return true;

}

void drone_interface_t::execute_trajectory(quadrotor_msgs::PolynomialTrajectory const & traj, bool do_not_check ) const {
  if ( do_not_check || ready(true) ) {
    pub_traj.publish(traj);
  }
}

void drone_interface_t::setup_trajectory_generator() {
  traj_generator = create_setting(max_vel,max_acc);
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

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan(Vector3d const & tgt ) const {
  return plan(traj_generator,tgt);
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan(Vector3d const & tgt, double yaw ) const {
  return plan(traj_generator,tgt,yaw);
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan(const vector<Vector3d> & waypoints_in ) const {
  return plan(traj_generator,waypoints_in);
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan(vector<Vector3d> const & waypoints_in, double final_yaw ) const {
  return plan(traj_generator,waypoints_in,final_yaw);
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan( AmTraj const & generator, vector<Vector3d> const & waypoints_in, double final_yaw ) const {

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
    return plan(waypoints_out.back());
  }

  Vector3d v0 = Vector3d::Zero();
  auto traj = generator.genOptimalTrajDTC(waypoints_out, v0, v0, v0, v0);
  return to_ros_msg(traj, uav_yaw_from_odom(latest_odom), final_yaw, ros::Time::now());

}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan( AmTraj const & generator, Vector3d const & tgt, double yaw ) const {

  Vector3d from = uav_utils::from_point_msg(latest_odom.pose.pose.position);

  double dist = (tgt - from).norm();

  if ( dist < epsi_distance ) {
    return gen_setpoint_traj(latest_odom, tgt, yaw, 0.5);
  }
  else {
    vector<Vector3d> waypoints;
    waypoints.emplace_back(from);
    waypoints.emplace_back((from + tgt) / 2);
    waypoints.emplace_back(tgt);
    Vector3d v0 = Vector3d::Zero();
    auto traj = generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
    return to_ros_msg(traj,uav_yaw_from_odom(latest_odom),yaw,ros::Time::now());
  }

}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan( AmTraj const & generator, vector<Vector3d> const & waypoints_in ) const {
  return plan(generator,waypoints_in,uav_yaw_from_odom(latest_odom));
}

quadrotor_msgs::PolynomialTrajectory drone_interface_t::plan( AmTraj const & generator, Vector3d const & tgt ) const {
  return plan(generator,tgt,uav_yaw_from_odom(latest_odom));
}

void drone_interface_t::go_to_point( const Vector3d & target) const {

  if (!ready()) return;

  execute_trajectory(plan(target));

}

void drone_interface_t::go_to_point_in_intermediate_frame( const Vector3d & point ) const {

  if (!ready()) return;

  Vector3d target = point_in_intermediate_frame(point,latest_odom);

  execute_trajectory(plan(target));

}

void drone_interface_t::follow_waypoints(const vector<Vector3d> &waypoints) const {

  if (!ready()) return;

  execute_trajectory(plan(waypoints));

}

void drone_interface_t::follow_waypoints( vector<Vector3d> const & waypoints, double yaw ) const {

  if (!ready()) return;

  execute_trajectory(plan(waypoints, yaw));

}

void drone_interface_t::follow_waypoints_in_intermediate_frame(const vector<Vector3d> & points) const {

  if (!ready()) return;

  vector<Vector3d> waypoints = points_in_intermediate_frame(points,latest_odom);

  execute_trajectory(plan(waypoints));

}

bool drone_interface_t::odom_in_time() const {
  return message_in_time(latest_odom,message_timeout);
}

AmTraj drone_interface_t::create_setting( double max_vel, double max_acc ) {
  return AmTraj(1024, 16, 0.4, max_vel, max_acc, 23, 0.02);
}

void drone_interface_t::reset_traj() const {
  quadrotor_msgs::PolynomialTrajectory msg;
  msg.header.stamp = ros::Time::now();
  msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;
  pub_traj.publish(msg);
}

bool drone_interface_t::n3ctrl_accept_traj( n3ctrl::N3CtrlState const & msg ) {
  return msg.state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER;
}

void drone_interface_t::arm_motors() const {
  std_msgs::UInt8 msg;
  msg.data = trigger_arm_motors;
  pub_trigger.publish(msg);
  ROS_WARN("[%s] prepare to take off!", name.c_str());
}

Vector3d drone_interface_t::position() const {
  return uav_utils::from_point_msg(latest_odom.pose.pose.position);
}

void drone_interface_t::cmd_pos_vel( const Vector3d & pos, const Vector3d & vel ) const {
  quadrotor_msgs::PositionCommand rst;
  rst.header.stamp = ros::Time::now();
  rst.header.frame_id = "world";
  rst.position = uav_utils::to_point_msg(pos);
  rst.velocity = uav_utils::to_vector3_msg(vel);
  rst.acceleration = uav_utils::to_vector3_msg(Vector3d::Zero());
  rst.yaw = uav_yaw_from_odom(latest_odom);
  rst.yaw_dot = 0;
  pub_pos_vel_cmd.publish(rst);
}

void drone_interface_t::cmd_pos_vel_yaw( const Vector3d & pos, const Vector3d & vel, double yaw ) const {
  quadrotor_msgs::PositionCommand rst;
  rst.header.stamp = ros::Time::now();
  rst.header.frame_id = "world";
  rst.position = uav_utils::to_point_msg(pos);
  rst.velocity = uav_utils::to_vector3_msg(vel);
  rst.acceleration = uav_utils::to_vector3_msg(Vector3d::Zero());
  rst.yaw = yaw;
  rst.yaw_dot = 0;
  pub_pos_vel_cmd.publish(rst);
}

double drone_interface_t::yaw() const {
  return uav_yaw_from_odom(latest_odom);
}