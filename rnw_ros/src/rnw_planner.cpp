//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/rnw_planner.h"
#include <uav_utils/converters.h>

void rnw_planner_t::start_walking(){
  if ( fsm == cone_fsm_e::idle ) {
    ROS_ERROR_STREAM("[rnw] Can't start walking when object is idle!");
    is_walking = false;
  }
  else {
    ROS_INFO_STREAM("[rnw] Start walking");
    is_walking = true;
  }
}

void rnw_planner_t::stop_walking(){
  ROS_INFO_STREAM("[rnw] Stop walking");
  is_walking = false;
}

void rnw_planner_t::cmd_complete(){
  if ( rnw_cmd.fsm == rnw_cmd_t::fsm_executing ) {
    rnw_cmd.fsm = rnw_cmd_t::fsm_idle;
  }
}

bool rnw_planner_t::has_pending_cmd() const {
  return rnw_cmd.fsm == rnw_cmd_t::fsm_pending;
}

Vector3d rnw_planner_t::next_position() const {
  //return uav_utils::from_point_msg(latest_cmd.tip_setpoint);
  return rnw_cmd.setpoint_uav;
}

void rnw_planner_t::on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
  latest_cone_state = *msg;
  cone_state_init = true;
}

void rnw_planner_t::on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
  latest_uav_odom = *msg;
  uav_odom_init = true;
}

rnw_planner_t::rnw_planner_t( ros::NodeHandle & nh ){
  rnw_config.load_from_ros(nh);
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
  }

  if ( from != cone_fsm_e::idle && to == cone_fsm_e::idle ) {
    ROS_INFO_STREAM("[rnw] object became idle");
    stop_walking();
  }

  fsm = to;

}

void rnw_planner_t::on_debug_trigger( std_msgs::HeaderConstPtr const & msg ){
  ROS_WARN_STREAM("[rnw] Got debug trigger");
  if ( !is_walking ) {
    start_walking();
  }
  else {
    cmd_complete();
  }
}

void rnw_planner_t::spin(){

  ROS_INFO_STREAM("[rnw] planner main loop spinning");

  if ( uav_odom_init && cone_state_init ) {
    rnw_cmd.grip_state = grip_state_t(latest_cone_state,latest_uav_odom,rnw_config.flu_T_tcp);
    pub_grip_state.publish(rnw_cmd.grip_state.to_msg());
  }

  fsm_update();

  if ( rnw_cmd.fsm == rnw_cmd_t::fsm_pending ) {
    ROS_INFO_STREAM("[rnw] cmd pending, do not plan");
    return;
  }
  else if ( rnw_cmd.fsm == rnw_cmd_t::fsm_executing ) {
    ROS_INFO_STREAM("[rnw] cmd executing, do not plan");
    return;
  }

  if ( fsm == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] cmd idle and object q-static, plan next cmd");

    bool grip_bad = rnw_cmd.grip_state.grip_depth < 0.05;
    bool posture_bad = latest_cone_state.euler_angles.y < 30;

    if ( request_adjust_grip || grip_bad ) {
      request_adjust_grip = false;
      request_adjust_nutation = false;
      plan_cmd_adjust_grip();
    }
    else if ( request_adjust_nutation || posture_bad ) {
      request_adjust_grip = false;
      request_adjust_nutation = false;
      plan_cmd_adjust_nutation();
    }
    else {
      plan_cmd_walk();
    }

  }
  else if ( fsm == cone_fsm_e::idle ) {
    request_adjust_grip = false;
    request_adjust_nutation = false;
  }

}

void rnw_planner_t::trigger_adjust_grip(){
  request_adjust_grip = true;
}

void rnw_planner_t::trigger_adjust_nutation(){
  request_adjust_nutation = true;
}

void rnw_planner_t::plan_cmd_walk(){
  rot_dir = -rot_dir;
  Vector3d G = uav_utils::from_point_msg(latest_cone_state.contact_point);
  Vector3d apex = rnw_cmd.grip_state.grip_point;
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
  rnw_cmd.tau_vec = setpoint_apex - apex;
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
  rnw_cmd.tau_vec = Vector3d::Zero();
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
  rnw_cmd.tau_vec = Vector3d::Zero();
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_adjust_nutation;
  rnw_cmd.cmd_idx++;
  //rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;

}