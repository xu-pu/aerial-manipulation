#include "rnw_ros/rnw_planner_v2.h"

rnw_planner_v2_t::rnw_planner_v2_t( ros::NodeHandle & h, rnw_config_t const & cfg )
        : rnw_config(cfg), nh(h), precession_regulator(cfg) { }

void rnw_planner_v2_t::on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
  latest_cone_state = *msg;
  cone_state_init = true;
}

void rnw_planner_v2_t::spin(){

  //ROS_INFO_STREAM("[rnw] planner main loop spinning");

  if (!cone_state_init) {
    ROS_WARN("[rnw_planner] waiting for cone_state");
    return;
  }

  fsm_update();

  /// Finished updating states
  /////////////////////////////////
  /// Do Planning

  if ( cmd_fsm == cmd_fsm_e::idle && cone_fsm == cone_fsm_e::qstatic ) {
    //ROS_INFO_STREAM("[rnw_planner] plan next cmd");
    plan_next_cmd();
  }
  else if ( cmd_fsm == cmd_fsm_e::pending ) {
    //ROS_INFO_STREAM("[rnw_planner] cmd pending, do not plan");
  }
  else if ( cmd_fsm == cmd_fsm_e::executing ) {
    //ROS_INFO_STREAM("[rnw_planner] cmd executing, do not plan");
  }

}

void rnw_planner_v2_t::fsm_update(){
  if ( latest_cone_state.euler_angles.y < rnw_config.rnw.min_nutation_deg * deg2rad ) {
    fsm_transition(cone_fsm, cone_fsm_e::idle);
  }
//  else if ( !latest_cone_state.is_point_contact ){
//    fsm_transition(fsm, cone_fsm_e::idle);
//  }
  else if ( abs(latest_cone_state.euler_angles_velocity.z) < rnw_config.rnw.ang_vel_threshold ) {
    fsm_transition(cone_fsm, cone_fsm_e::qstatic);
  }
  else {
    fsm_transition(cone_fsm, cone_fsm_e::rocking);
  }
}

void rnw_planner_v2_t::fsm_transition(cone_fsm_e from, cone_fsm_e to ){

  if ( from == cone_fsm_e::rocking && to == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] from rocking to qstatic");
  }

  if ( to == cone_fsm_e::rocking && from == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] from qstatic to rocking");
  }

  if ( to == cone_fsm_e::idle ) {
    cmd_fsm = cmd_fsm_e::idle;
    step_count = 0;
  }

  if ( from != cone_fsm_e::idle && to == cone_fsm_e::idle ) {
    ROS_INFO_STREAM("[rnw] object became idle");
    stop_walking();
  }

  cone_fsm = to;

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
    //pub_walking_state = ros::Publisher();
    //ROS_INFO_STREAM("[rnw] Stop walking");
  }
}

void rnw_planner_v2_t::plan_next_cmd(){
  if ( is_walking ) {
    ROS_INFO_STREAM("[rnw_planner] plan next step of r-n-w");
    plan_cmd_walk();
    //plan_cmd_walk_corridor();
    //plan_cmd_walk_no_feedforward();
  }
}

void rnw_planner_v2_t::plan_cmd_walk(){

  // adjust nutation first

  Vector3d G = uav_utils::from_point_msg(latest_cone_state.contact_point);
  Vector3d D = uav_utils::from_point_msg(latest_cone_state.disc_center);
  Vector3d Dg = D; Dg.z() = rnw_config.ground_z;

  Vector3d e1 = (Dg-G).normalized();
  Vector3d e2 = Vector3d::UnitZ();
  Vector3d K = e1.cross(e2);
  Vector3d C = point_at_grip_depth(latest_cone_state,rnw_config.rnw.desired_grip_depth);

  // make sure they are radiant
  double cur_nutation = latest_cone_state.euler_angles.y;
  double desired_nutation = rnw_config.rnw.desired_nutation*deg2rad;
  double theta = desired_nutation - cur_nutation;
  // rotate along K, positive rotation increase nutation
  Vector3d C_prime = rotate_point_along_axis(C,G,K,theta);

  // left-right step

  rot_dir = -rot_dir;

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
  rnw_command.cmd_idx++;
  step_count++;
  cmd_fsm = cmd_fsm_e::pending;

  precession_regulator.step(latest_cone_state);

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
  desired_heading_yaw = cone_yaw(cone_state);
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
    desired_heading_yaw += ( deg2rad * rnw_config.rnw.lap_ang_vel_deg );
    desired_heading_yaw = uav_utils::normalize_angle(desired_heading_yaw);
  }

  if ( step_count > 1 ) {
    // update heading direction
    double diff_odd = uav_utils::normalize_angle(cone_yaw(last_step_odd) - desired_heading_yaw);
    double diff_even = uav_utils::normalize_angle(cone_yaw(last_step_even) - desired_heading_yaw);
    cur_relative_yaw = ( diff_odd + diff_even ) / 2;

    ROS_ERROR_STREAM("[rnw_planner] heading direction error " << cur_relative_yaw*rad2deg << " deg");
  }
  step_count++;

  last_step = cone_state;

}