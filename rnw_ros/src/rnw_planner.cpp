//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/rnw_planner.h"
#include <uav_utils/converters.h>

void rnw_planner_t::start_planning_cmd(){
  if ( fsm == cone_fsm_e::idle ) {
    ROS_ERROR_STREAM("[rnw] Can't start planning when object is idle!");
    plan_cmd = false;
  }
  else {
    ROS_INFO_STREAM("[rnw] Start planning rocking commands");
    plan_cmd = true;
  }
}

void rnw_planner_t::stop_planning_cmd(){
  ROS_INFO_STREAM("[rnw] Stop planning rocking commands");
  plan_cmd = false;
}

void rnw_planner_t::cmd_ack(){
  cmd_pending = false;
}

bool rnw_planner_t::has_pending_cmd() const {
  return cmd_pending;
}

Vector3d rnw_planner_t::next_position() const {
  return uav_utils::from_point_msg(latest_cmd.tip_setpoint);
}

void rnw_planner_t::on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){

  latest_cone_state = *msg;

  update_state(latest_cone_state);

  switch ( fsm ) {
    case cone_fsm_e::idle:
      break;
    case cone_fsm_e::rocking:
      break;
    case cone_fsm_e::qstatic:
      plan_next_position();
      break;
    default:
      ROS_ERROR_STREAM("[rnw_planner] Invalid Cone State");
      break;
  }

}

rnw_planner_t::rnw_planner_t( ros::NodeHandle & nh ){
  pub_rocking_cmd = nh.advertise<rnw_msgs::RockingCmd>("/rnw/rocking_cmd",10);
}

void rnw_planner_t::plan_next_position(){

  if ( !plan_cmd ) {
    return;
  }

  if ( cmd_pending ) {
    latest_cmd.header.stamp = ros::Time::now();
    pub_rocking_cmd.publish(latest_cmd);
    return;
  }

  if ( !latest_cone_state.is_point_contact ) {
    ROS_ERROR_STREAM("[rnw] trying to plan but contact point is invalid!");
    return;
  }

  rot_dir = -rot_dir;

  Vector3d contact_point = uav_utils::from_point_msg(latest_cone_state.contact_point);
  Vector3d apex = uav_utils::from_point_msg(latest_cone_state.tip);
  Vector3d v = apex - contact_point;
  Matrix3d offset = Eigen::AngleAxisd( rot_amp_deg*deg2rad*rot_dir, Vector3d::UnitZ() ).toRotationMatrix();
  Vector3d next_v = offset * v;
  Vector3d next_tip = contact_point + next_v;

  latest_cmd.header.stamp = latest_cone_state.header.stamp;
  latest_cmd.step_count = step_count;
  latest_cmd.tip_setpoint = uav_utils::to_point_msg(next_tip);
  latest_cmd.cmd_idx = cmd_idx++;

  pub_rocking_cmd.publish(latest_cmd);

  cmd_pending = true;

}

void rnw_planner_t::update_state( rnw_msgs::ConeState const & msg ){

  if ( msg.euler_angles.y < min_tilt ) {
    state_transition(fsm,cone_fsm_e::idle);
  }
  else if ( !msg.is_point_contact ){
    state_transition(fsm,cone_fsm_e::idle);
  }
  else if ( abs(msg.euler_angles_velocity.z) < ang_vel_threshold ) {
    state_transition(fsm,cone_fsm_e::qstatic);
  }
  else {
    state_transition(fsm,cone_fsm_e::rocking);
  }

}

void rnw_planner_t::state_transition( cone_fsm_e from, cone_fsm_e to ){

  if ( from == cone_fsm_e::rocking && to == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] from rocking to qstatic");
    step_count++;
  }

  if ( to == cone_fsm_e::rocking && from == cone_fsm_e::qstatic ) {
    ROS_INFO_STREAM("[rnw] from qstatic to rocking");
  }

  if ( to == cone_fsm_e::idle ) {
    cmd_pending = false;
    step_count = 0;
  }

  if ( from != cone_fsm_e::idle && to == cone_fsm_e::idle ) {
    ROS_INFO_STREAM("[rnw] object became idle");
    stop_planning_cmd();
  }

  fsm = to;

}

void rnw_planner_t::on_debug_trigger( std_msgs::HeaderConstPtr const & msg ){
  ROS_WARN_STREAM("[rnw] Got debug trigger");
  if ( !plan_cmd ) {
    start_planning_cmd();
  }
  else {
    cmd_ack();
  }
}
