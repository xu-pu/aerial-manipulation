//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/rnw_planner.h"
#include <uav_utils/converters.h>

void rnw_planner_t::start_walking(){
  if ( fsm == cone_fsm_e::idle ) {
    ROS_ERROR_STREAM("[rnw] Can't start walking when object is idle!");
    rnw_cmd.is_walking = false;
  }
  else {
    rnw_cmd.is_walking = true;
    rnw_cmd.walk_idx++;
    stringstream ss; ss << "/rnw/walking_state/session_" << rnw_cmd.walk_idx;
    pub_walking_state = nh.advertise<rnw_msgs::WalkingState>(ss.str(),100);
    ROS_INFO_STREAM("[rnw] Start walking, #" << rnw_cmd.walk_idx << ", topic: " << ss.str());
  }
}

void rnw_planner_t::stop_walking(){
  if ( rnw_cmd.is_walking ) {
    ROS_INFO_STREAM("[rnw] Stop walking");
    rnw_cmd.is_walking = false;
  }
}

void rnw_planner_t::cmd_complete(){
  if ( rnw_cmd.fsm == rnw_cmd_t::fsm_executing ) {
    rnw_cmd.fsm = rnw_cmd_t::fsm_idle;
  }
}

void rnw_planner_t::on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
  latest_cone_state = *msg;
  cone_state_init = true;
}

void rnw_planner_t::on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
  latest_uav_odom = *msg;
  uav_odom_init = true;
}

rnw_planner_t::rnw_planner_t( ros::NodeHandle & h, rnw_config_t const & cfg ) : rnw_config(cfg), nh(h) {
  pub_rocking_cmd = nh.advertise<rnw_msgs::RockingCmd>("/rnw/rocking_cmd",10);
  pub_grip_state = nh.advertise<rnw_msgs::GripState>("/rnw/grip_state",10);
}

void rnw_planner_t::fsm_update(){
  if ( latest_cone_state.euler_angles.y < min_tilt ) {
    fsm_transition(fsm, cone_fsm_e::idle);
  }
  else if ( !latest_cone_state.is_point_contact ){
    fsm_transition(fsm, cone_fsm_e::idle);
  }
  else if ( abs(latest_cone_state.euler_angles_velocity.z) < ang_vel_threshold ) {
    fsm_transition(fsm, cone_fsm_e::qstatic);
  }
  else {
    fsm_transition(fsm, cone_fsm_e::rocking);
  }
}

void rnw_planner_t::fsm_transition(cone_fsm_e from, cone_fsm_e to ){

  if ( from == cone_fsm_e::rocking && to == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] from rocking to qstatic");
  }

  if ( to == cone_fsm_e::rocking && from == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] from qstatic to rocking");
  }

  if ( to == cone_fsm_e::idle ) {
    rnw_cmd.fsm = rnw_cmd_t::fsm_idle;
    rnw_cmd.step_count = 0;
    request_adjust_grip = false;
    request_adjust_nutation = false;
  }

  if ( from != cone_fsm_e::idle && to == cone_fsm_e::idle ) {
    ROS_INFO_STREAM("[rnw] object became idle");
    stop_walking();
  }

  fsm = to;

}

void rnw_planner_t::on_debug_trigger( std_msgs::HeaderConstPtr const & msg ){
  ROS_WARN_STREAM("[rnw] Got debug trigger");
  if ( !rnw_cmd.is_walking ) {
    start_walking();
  }
  else {
    cmd_complete();
  }
}

void rnw_planner_t::spin(){

  //ROS_INFO_STREAM("[rnw] planner main loop spinning");

  if (!(uav_odom_init&&cone_state_init)) {
    ROS_WARN("[rnw_planner] waiting for uav_odom and cone_state");
    return;
  }

  rnw_cmd.grip_state = grip_state_t(latest_cone_state,latest_uav_odom,rnw_config.flu_T_tcp);
  pub_grip_state.publish(rnw_cmd.grip_state.to_msg());

  if ( rnw_cmd.grip_state.grip_valid ) {
    rnw_cmd.err_grip_depth = rnw_cmd.grip_state.grip_depth - rnw_config.rnw.desired_grip_depth;
    rnw_cmd.err_nutation_deg = latest_cone_state.euler_angles.y * rad2deg - rnw_config.rnw.desired_nutation;
  }
  else {
    rnw_cmd.err_grip_depth = 0;
    rnw_cmd.err_nutation_deg = 0;
  }

  fsm_update();

  /// Finished updating states
  /////////////////////////////////
  /// Do Planning

  if ( rnw_cmd.grip_state.grip_valid && rnw_cmd.fsm == rnw_cmd_t::fsm_idle && fsm == cone_fsm_e::qstatic ) {
    //ROS_INFO_STREAM("[rnw_planner] plan next cmd");
    plan_next_cmd();
  }
  else if ( rnw_cmd.fsm == rnw_cmd_t::fsm_pending ) {
    //ROS_INFO_STREAM("[rnw_planner] cmd pending, do not plan");
  }
  else if ( rnw_cmd.fsm == rnw_cmd_t::fsm_executing ) {
    //ROS_INFO_STREAM("[rnw_planner] cmd executing, do not plan");
  }

  pub_rocking_cmd.publish(rnw_cmd.to_msg());

}

void rnw_planner_t::trigger_adjust_grip(){
  if ( rnw_cmd.grip_state.initialized && rnw_cmd.grip_state.grip_valid && fsm != cone_fsm_e::idle ) {
    ROS_INFO("[rnw_planner] grip adjustment request accepted");
    request_adjust_grip = true;
  }
  else {
    ROS_ERROR("[rnw_planner] Can not adjust grip at current state!");
  }
}

void rnw_planner_t::trigger_adjust_nutation(){
  if ( rnw_cmd.grip_state.initialized && rnw_cmd.grip_state.grip_valid && fsm != cone_fsm_e::idle ) {
    ROS_INFO("[rnw_planner] nutation adjustment request accepted");
    request_adjust_nutation = true;
  }
  else {
    ROS_ERROR("[rnw_planner] Can not adjust nutation at current state!");
  }
}

void rnw_planner_t::plan_cmd_walk(){
  rot_dir = -rot_dir;
  Vector3d G = uav_utils::from_point_msg(latest_cone_state.contact_point);
  //Vector3d apex = rnw_cmd.grip_state.grip_point;
  Vector3d apex = point_at_grip_depth(latest_cone_state,rnw_config.rnw.desired_grip_depth);
  Vector3d v = apex - G;
  Matrix3d rot = Eigen::AngleAxisd( rot_amp_deg*deg2rad*rot_dir, Vector3d::UnitZ() ).toRotationMatrix();
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;
  Vector3d setpoint_uav = tcp2uav(setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);

  rnw_cmd.setpoint_uav = setpoint_uav;
  rnw_cmd.setpoint_apex = setpoint_apex;
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = rot_amp_deg;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_rocking;
  rnw_cmd.cmd_idx++;
  rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;

}

void rnw_planner_t::plan_cmd_adjust_grip(){
  rnw_cmd.setpoint_apex = point_at_grip_depth(latest_cone_state,rnw_config.rnw.desired_grip_depth);
  rnw_cmd.setpoint_uav = tcp2uav(rnw_cmd.setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = 0;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_adjust_grip;
  rnw_cmd.cmd_idx++;
  //rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;
}

void rnw_planner_t::plan_cmd_adjust_nutation(){

  Vector3d O = uav_utils::from_point_msg(latest_cone_state.contact_point);
  Vector3d D = uav_utils::from_point_msg(latest_cone_state.disc_center);
  Vector3d Dg = D; Dg.z() = rnw_config.ground_z;

  Vector3d e1 = (Dg-O).normalized();
  Vector3d e2 = Vector3d::UnitZ();
  Vector3d K = e1.cross(e2);

  // rotate along K, positive rotation increase nutation

  Vector3d C = rnw_cmd.grip_state.grip_point;
  // make sure they are radiant
  double cur_nutation = latest_cone_state.euler_angles.y;
  double desired_nutation = rnw_config.rnw.desired_nutation*deg2rad;
  double theta = desired_nutation - cur_nutation;

  rnw_cmd.setpoint_apex = rotate_point_along_axis(C,O,K,theta);
  rnw_cmd.setpoint_uav = tcp2uav(rnw_cmd.setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = 0;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_adjust_nutation;
  rnw_cmd.cmd_idx++;
  //rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;

}

void rnw_planner_t::plan_cmd_walk_with_nutation_adjustment(){

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

  Vector3d v = C_prime - G;
  Matrix3d rot = Eigen::AngleAxisd( rot_amp_deg*deg2rad*rot_dir, Vector3d::UnitZ() ).toRotationMatrix();
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;
  Vector3d setpoint_uav = tcp2uav(setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);

  rnw_cmd.setpoint_uav = setpoint_uav;
  rnw_cmd.setpoint_apex = setpoint_apex;
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = rot_amp_deg;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_rocking;
  rnw_cmd.cmd_idx++;
  rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;

}

void rnw_planner_t::plan_next_cmd(){

  bool grip_bad = abs(rnw_cmd.err_grip_depth) > rnw_config.rnw.adjust_grip_depth_threshold;

  bool posture_bad = abs(rnw_cmd.err_nutation_deg) > rnw_config.rnw.adjust_nutation_threshold;

  grip_bad = false; // disable separate grip adjust

  posture_bad = false; // disable separate nutation adjustment

  if ( rnw_cmd.is_walking ) {

    if ( request_adjust_grip || grip_bad ) {
      ROS_WARN_STREAM("[rnw_planner] adjusting grip depth");
      plan_cmd_adjust_grip();
    }
    else if ( request_adjust_nutation || posture_bad ) {
      ROS_WARN_STREAM("[rnw_planner] adjusting nutation");
      plan_cmd_adjust_nutation();
    }
    else {
      ROS_INFO_STREAM("[rnw_planner] plan next step of r-n-w");
      //plan_cmd_walk();
      plan_cmd_walk_with_nutation_adjustment();
    }

  }
  else if ( request_adjust_grip ) {
    plan_cmd_adjust_grip();
  }
  else if ( request_adjust_nutation ) {
    plan_cmd_adjust_nutation();
  }

  request_adjust_grip = false;
  request_adjust_nutation = false;

}

rnw_cmd_t * rnw_planner_t::take_cmd(){
  switch ( rnw_cmd.fsm ) {
    case rnw_cmd_t::fsm_idle:
      ROS_ERROR("[rnw] there is no command at the moment");
      break;
    case rnw_cmd_t::fsm_pending:
      ROS_INFO_STREAM("[rnw] command is taken, now executing");
      rnw_cmd.fsm = rnw_cmd_t::fsm_executing;
      break;
    case rnw_cmd_t::fsm_executing:
      ROS_ERROR("[rnw] command is already taken!");
      break;
    default:
      ROS_ERROR("[rnw] invalid rnw_cmd state!");
  }
  return &rnw_cmd;
}

rnw_msgs::RockingCmd rnw_cmd_t::to_msg() const {
  rnw_msgs::RockingCmd msg;
  msg.header.stamp = ros::Time::now();
  msg.fsm = fsm;
  msg.cmd_type = cmd_type;
  msg.cmd_idx = cmd_idx;
  msg.grip_state = grip_state.to_msg();
  msg.setpoint_uav = uav_utils::to_point_msg(setpoint_uav);
  msg.setpoint_apex = uav_utils::to_point_msg(setpoint_apex);
  msg.setpoint_nutation_deg = setpoint_nutation;
  msg.setpoint_grip_depth = setpoint_grip_depth;
  msg.err_nutation_deg = err_nutation_deg;
  msg.err_grip_depth = err_grip_depth;
  msg.tau_deg = tau_deg;
  msg.step_count = step_count;
  msg.is_walking = is_walking;
  msg.walk_idx = walk_idx;
  return msg;
}