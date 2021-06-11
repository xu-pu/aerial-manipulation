#include "rnw_ros/rnw_planner_v2.h"

rnw_planner_v2_t::rnw_planner_v2_t( rnw_config_t const & cfg )
        : rnw_config(cfg), precession_regulator(cfg)
{
  ros::NodeHandle nh("~");
  pub_rnw_state = nh.advertise<rnw_msgs::RnwState>("/rnw/state",100);
}

void rnw_planner_v2_t::on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
  latest_cone_state = *msg;
  cone_state_init = true;
}

void rnw_planner_v2_t::spin(){

  control_loop();

  pub_rnw_state.publish(to_rnw_state());

}

void rnw_planner_v2_t::start_walking(){
  if (cone_fsm == cone_fsm_e::idle ) {
    ROS_ERROR_STREAM("[rnw] Can't start walking when object is idle!");
    is_walking = false;
  }
  else if ( !is_walking ) {
    precession_regulator.start(latest_cone_state);
    is_walking = true;
    walk_idx++;
    step_count = 0;
  }
}

void rnw_planner_v2_t::stop_walking(){
  if ( is_walking ) {
    precession_regulator.end();
    is_walking = false;
    step_count = 0;
  }
}

rnw_msgs::RnwState rnw_planner_v2_t::to_rnw_state() const {
  rnw_msgs::RnwState msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.is_walking = is_walking;
  msg.step_count = step_count;
  msg.setpoint = uav_utils::to_point_msg(rnw_command.control_point_setpoint);
  return msg;
}

void rnw_planner_v2_t::control_loop(){

  if (!cone_state_init) {
    ROS_WARN("[rnw_planner] waiting for cone_state");
    return;
  }

  if ( !is_walking ) {
    return;
  }

  if ( latest_cone_state.euler_angles.y < rnw_config.rnw.min_nutation_deg * deg2rad ) {
    stop_walking();
  }

  // avoid transient states
  if ( ros::Time::now() - rnw_command.stamp < ros::Duration(rnw_config.rnw.min_step_interval) ) {
    return;
  }

  if ( abs(latest_cone_state.euler_angles_velocity.z) > rnw_config.rnw.ang_vel_threshold
       || abs(latest_cone_state.euler_angles_velocity.x) > rnw_config.rnw.ang_vel_threshold ) {
    return;
  }

  /**
   * if phi > epsi, make sure rot direction match, otherwise ignore the direction condition
   * this will allow initialization
   */

  double epsi_phi = 10 * deg2rad;

  bool direction_matched = -1 * rot_dir * latest_cone_state.euler_angles.z > epsi_phi;

  bool low_energy = std::abs(latest_cone_state.euler_angles.z) < epsi_phi;

  if ( direction_matched || low_energy ) {
    plan_cmd_walk();
  }

}

void rnw_planner_v2_t::plan_cmd_walk(){

  rnw_command.stamp = ros::Time::now();

  // adjust nutation first

  Vector3d G = uav_utils::from_point_msg(latest_cone_state.contact_point);
  Vector3d D = uav_utils::from_point_msg(latest_cone_state.disc_center);
  Vector3d Dg = D; Dg.z() = rnw_config.ground_z;

  Vector3d e1 = (Dg-G).normalized();
  Vector3d e2 = Vector3d::UnitZ();
  Vector3d K = e1.cross(e2);
  Vector3d C = point_at_grip_depth(latest_cone_state,rnw_config.caging.desired_grip_depth);

  // make sure they are radiant
  double cur_nutation = latest_cone_state.euler_angles.y;
  double desired_nutation = rnw_config.rnw.desired_nutation*deg2rad;
  double theta = desired_nutation - cur_nutation;
  // rotate along K, positive rotation increase nutation
  Vector3d C_prime = rotate_point_along_axis(C,G,K,theta);

  // left-right step

  double steering_term = 0;
  if ( rnw_config.rnw.enable_steering ) {
    steering_term = - rnw_config.rnw.yaw_gain * precession_regulator.cur_relative_yaw;
  }

  double rot_rad = rot_dir * ( rnw_config.rnw.tau * deg2rad ) + steering_term;

  Matrix3d rot = Eigen::AngleAxisd(rot_rad,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d v = C_prime - G;
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;

  rnw_command.control_point_setpoint = setpoint_apex;
  rnw_command.heading = precession_regulator.desired_heading;
  rnw_command.cmd_idx++;
  step_count++;
  cmd_fsm = cmd_fsm_e::pending;

  precession_regulator.step(latest_cone_state);

  rot_dir = -rot_dir;

}

rnw_command_t rnw_planner_v2_t::take_cmd(){
  switch ( cmd_fsm ) {
    case cmd_fsm_e::idle:
      ROS_ERROR("[rnw] there is no command at the moment");
      break;
    case cmd_fsm_e::pending:
      ROS_INFO_STREAM("[rnw] command is taken, now executing, step #" << step_count);
      cmd_fsm = cmd_fsm_e::executing;
      break;
    case cmd_fsm_e::executing:
      ROS_ERROR("[rnw] command is already taken!");
      break;
    default:
      ROS_ERROR("[rnw] invalid rnw_cmd state!");
  }
  return rnw_command;
}

void rnw_planner_v2_t::cmd_complete(){
  if ( cmd_fsm == cmd_fsm_e::executing ) {
    cmd_fsm = cmd_fsm_e::idle;
  }
}

precession_regulator_t::precession_regulator_t(rnw_config_t const & c ) : rnw_config(c) {}

void precession_regulator_t::start(rnw_msgs::ConeState const & cone_state ){
  if ( rnw_config.rnw.specify_heading ) {
    desired_heading = rnw_config.rnw.heading;
    ROS_INFO("[rnw_planner] heading direction specified in config");
  }
  else  {
    desired_heading = cone_yaw(cone_state);
    ROS_INFO("[rnw_planner] did not specify heading direction, use the current value");
  }
  ROS_INFO("[rnw_planner] heading=%d",(int)(desired_heading * rad2deg));
  cur_relative_yaw = 0;
  step_count = 0;
  last_step = cone_state;
}

void precession_regulator_t::end(){}

void precession_regulator_t::step(rnw_msgs::ConeState const & cone_state ){

  if ( step_count % 2 == 0 ) {
    last_step_even = cone_state;
  }
  else {
    last_step_odd = cone_state;
  }

  if ( step_count >= rnw_config.rnw.lap_start ) {
    desired_heading += (deg2rad * rnw_config.rnw.lap_ang_vel_deg );
    desired_heading = uav_utils::normalize_angle(desired_heading);
  }

  if ( step_count > 1 ) {
    // update heading direction
    double diff_odd = uav_utils::normalize_angle(cone_yaw(last_step_odd) - desired_heading);
    double diff_even = uav_utils::normalize_angle(cone_yaw(last_step_even) - desired_heading);
    cur_relative_yaw = ( diff_odd + diff_even ) / 2;

    ROS_ERROR_STREAM("[rnw_planner] heading direction error " << cur_relative_yaw*rad2deg << " deg");
  }
  step_count++;

  last_step = cone_state;

}

rnw_msgs::RnwCmd rnw_command_t::to_ros_msg() const {
  rnw_msgs::RnwCmd msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.cmd_idx = cmd_idx;
  msg.setpoint = uav_utils::to_point_msg(control_point_setpoint);
  return msg;
}