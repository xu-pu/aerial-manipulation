//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/rnw_planner.h"
#include "rnw_ros/rnw_utils.h"
#include <uav_utils/converters.h>

void rnw_planner_t::start_walking(){
  if ( fsm == cone_fsm_e::idle ) {
    ROS_ERROR_STREAM("[rnw] Can't start walking when object is idle!");
    rnw_cmd.is_walking = false;
  }
  else if ( !rnw_cmd.is_walking ) {
    walking_state.start(latest_cone_state);
    energy_feedback.init(latest_cone_state);
    corridor.init(latest_cone_state);
    rnw_cmd.desired_yaw = walking_state.desired_uav_yaw();
    rnw_cmd.is_walking = true;
    rnw_cmd.walk_idx++;
    rnw_cmd.step_count = 0;
    request_adjust_nutation = true;
    request_adjust_grip = true;
    integration_term = 0;
    stringstream ss; ss << "/rnw/walking_state/session_" << int(rnw_cmd.walk_idx);
    pub_walking_state = nh.advertise<rnw_msgs::WalkingState>(ss.str(),100);
    ROS_WARN_STREAM("[rnw] Start walking, " << pub_walking_state.getTopic());
  }
}

void rnw_planner_t::stop_walking(){
  if ( rnw_cmd.is_walking ) {
    walking_state.end();
    rnw_cmd.is_walking = false;
    pub_walking_state = ros::Publisher();
    ROS_INFO_STREAM("[rnw] Stop walking");
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

rnw_planner_t::rnw_planner_t( ros::NodeHandle & h, rnw_config_t const & cfg ) : rnw_config(cfg), nh(h), walking_state(cfg), corridor(cfg) {
  pub_rocking_cmd = nh.advertise<rnw_msgs::RockingCmd>("/rnw/rocking_cmd",10);
  pub_grip_state = nh.advertise<rnw_msgs::GripState>("/rnw/grip_state",10);
}

void rnw_planner_t::fsm_update(){
  if ( latest_cone_state.euler_angles.y < rnw_config.rnw.min_nutation_deg * deg2rad ) {
    fsm_transition(fsm, cone_fsm_e::idle);
  }
//  else if ( !latest_cone_state.is_point_contact ){
//    fsm_transition(fsm, cone_fsm_e::idle);
//  }
  else if ( abs(latest_cone_state.euler_angles_velocity.z) < rnw_config.rnw.ang_vel_threshold ) {
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

  if ( rnw_cmd.is_walking ) {
    rnw_msgs::WalkingState msg;
    msg.header = latest_cone_state.header;
    msg.cone_state = latest_cone_state;
    msg.step_count = rnw_cmd.step_count;
    msg.tau_deg = rnw_config.rnw.tau;
    msg.desired_nutation_deg = rnw_config.rnw.desired_nutation;
    pub_walking_state.publish(msg);
  }

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

void rnw_planner_t::plan_cmd_adjust_grip(){
  request_adjust_grip = false;
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

  request_adjust_nutation = false;

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

void rnw_planner_t::plan_cmd_walk(){

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
    steering_term = - rnw_config.rnw.yaw_gain * walking_state.cur_relative_yaw;
  }

  // energy feedback term
  double energy_term = 0;
  if ( rnw_config.rnw.enable_energy_feedback ) {
    energy_feedback.step(latest_cone_state);
    energy_term = rnw_config.rnw.EKp * energy_feedback.E_dot;
  }

  double rot_rad = rot_dir * ( rnw_config.rnw.tau * deg2rad - energy_term ) + steering_term;

  Matrix3d rot = Eigen::AngleAxisd(rot_rad,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d v = C_prime - G;
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;
  Vector3d setpoint_uav = tcp2uav(setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);

  rnw_cmd.setpoint_uav = setpoint_uav;
  rnw_cmd.setpoint_apex = setpoint_apex;
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = rnw_config.rnw.tau;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_rocking;
  rnw_cmd.cmd_idx++;
  rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;

  if ( rnw_config.rnw.enable_steering ) {
    rnw_cmd.desired_yaw = walking_state.desired_uav_yaw();
  }
  else {
    double obj_heading = calc_obj_heading(latest_cone_state,walking_state.last_step);
    ROS_INFO_STREAM("[rnw] object heading dir " << obj_heading);
    rnw_cmd.desired_yaw = uav_yaw_from_cone_yaw(obj_heading);
  }

  walking_state.step(latest_cone_state);

}

void rnw_planner_t::plan_cmd_walk_corridor(){

  // left-right step
  rot_dir = -rot_dir;
  corridor.update_cone_state(latest_cone_state);
  Vector3d setpoint_apex = corridor.calc_next_c(latest_cone_state,(int)rot_dir);
  Vector3d setpoint_uav = tcp2uav(setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);

  rnw_cmd.setpoint_uav = setpoint_uav;
  rnw_cmd.setpoint_apex = setpoint_apex;
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = rnw_config.rnw.tau;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_rocking;
  rnw_cmd.cmd_idx++;
  rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;
  rnw_cmd.desired_yaw = uav_yaw_from_cone_yaw(corridor.corridor_dir);

  walking_state.step(latest_cone_state);

}

void rnw_planner_t::plan_cmd_walk_no_feedforward(){

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

  double constexpr max_int_term = M_PI_2;

  double yaw_term = - rnw_config.rnw.yaw_gain * walking_state.cur_relative_yaw;

  double desired_spin = deg2rad * rnw_config.rnw.desired_spin_deg;
  double spin_err = desired_spin - abs(latest_cone_state.euler_angles.z);
  integration_term += spin_err * rnw_config.rnw.spin_Ki;
  integration_term = max(min(integration_term, max_int_term), 0.); // constrain integration term to [0,PI/2]
  ROS_ERROR_STREAM("int_err " << integration_term << ", spin_err: " << spin_err);

  double PI_term = rnw_config.rnw.spin_Kp * spin_err + integration_term;

  double rot_rad = rot_dir * PI_term + yaw_term;

  Matrix3d rot = Eigen::AngleAxisd(rot_rad,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d v = C_prime - G;
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;
  Vector3d setpoint_uav = tcp2uav(setpoint_apex,latest_uav_odom,rnw_config.flu_T_tcp);

  rnw_cmd.setpoint_uav = setpoint_uav;
  rnw_cmd.setpoint_apex = setpoint_apex;
  rnw_cmd.setpoint_grip_depth = rnw_config.rnw.desired_grip_depth;
  rnw_cmd.setpoint_nutation = rnw_config.rnw.desired_nutation;
  rnw_cmd.tau_deg = rnw_config.rnw.tau;
  rnw_cmd.cmd_type = rnw_cmd_t::cmd_rocking;
  rnw_cmd.cmd_idx++;
  rnw_cmd.step_count++;
  rnw_cmd.fsm = rnw_cmd_t::fsm_pending;

  walking_state.step(latest_cone_state);

}

void rnw_planner_t::plan_next_cmd(){
  if ( request_adjust_grip ) {
    ROS_WARN_STREAM("[rnw_planner] adjusting grip depth");
    plan_cmd_adjust_grip();
  }
  else if ( request_adjust_nutation ) {
    ROS_WARN_STREAM("[rnw_planner] adjusting nutation");
    plan_cmd_adjust_nutation();
  }
  else if ( rnw_cmd.is_walking ) {
    ROS_INFO_STREAM("[rnw_planner] plan next step of r-n-w");
    plan_cmd_walk();
    //plan_cmd_walk_corridor();
    //plan_cmd_walk_no_feedforward();
  }
}

rnw_cmd_t * rnw_planner_t::take_cmd(){
  switch ( rnw_cmd.fsm ) {
    case rnw_cmd_t::fsm_idle:
      ROS_ERROR("[rnw] there is no command at the moment");
      break;
    case rnw_cmd_t::fsm_pending:
      ROS_INFO_STREAM("[rnw] command is taken, now executing, step #" << rnw_cmd.step_count);
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

void rnw_planner_t::trigger_adjust_yaw(){
  if ( rnw_cmd.is_walking ) {
    ROS_WARN_STREAM("[rnw_planner] yaw_adjustment triggered!");
    rnw_cmd.desired_yaw = uav_yaw_from_cone_state(latest_cone_state);
  }
  else {
    ROS_WARN_STREAM("[rnw_planner] Can't adjust yaw at current state");
  }
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

steering_controller_t::steering_controller_t(rnw_config_t const & c ) : rnw_config(c) {}

double steering_controller_t::desired_uav_yaw() const {
  return uav_yaw_from_cone_yaw(desired_heading_yaw);
}

void steering_controller_t::start(rnw_msgs::ConeState const & cone_state ){
  desired_heading_yaw = cone_yaw(cone_state);
  cur_relative_yaw = 0;
  step_count = 0;
  last_step = cone_state;
}

void steering_controller_t::end(){}

void steering_controller_t::step(rnw_msgs::ConeState const & cone_state ){

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

void energy_feedback_t::init( rnw_msgs::ConeState const & cone_state ){
  _init = true;
  step_count = 0;
  E = abs(cone_state.euler_angles.z);
  E_dot = 0;
  E_dot_dot = 0;
}

void energy_feedback_t::step( rnw_msgs::ConeState const & cone_state ){
  step_count++;
  double cur_E = abs(cone_state.euler_angles.z);
  double cur_E_dot = cur_E - E;
  E_dot_dot = cur_E_dot - E_dot;
  E_dot = cur_E_dot;
  E = cur_E;
}


corridor_controller_t::corridor_controller_t( rnw_config_t const & cfg ): config(cfg) {}

void corridor_controller_t::init( rnw_msgs::ConeState const & cone_state ){
  Vector3d C = point_at_grip_depth(cone_state,config.rnw.desired_grip_depth);
  corridor_origin = C;
  corridor_dir = cone_yaw(cone_state);
  corridor_width = config.rnw.tau_ff/2;
  double theta = corridor_dir;
  // setup the ref frame
  Eigen::Vector2d ex( cos(theta), sin(theta) );
  Eigen::Vector2d ey( -sin(theta), cos(theta) );
  Rwc.col(0) = ex;
  Rwc.col(1) = ey;
  Twc = Eigen::Vector2d( corridor_origin.x(), corridor_origin.y() );
  Rcw = Rwc.transpose();
  Tcw = -Rcw * Twc;
  _init = true;
}

void corridor_controller_t::update_cone_state( rnw_msgs::ConeState const & cone_state ){
  // no nothing for now
}

Vector3d corridor_controller_t::calc_next_c( rnw_msgs::ConeState const & cone_state, int dir ){

  // adjust nutation first

  Vector3d G = uav_utils::from_point_msg(cone_state.contact_point);
  Vector3d D = uav_utils::from_point_msg(cone_state.disc_center);
  Vector3d Dg = D; Dg.z() = config.ground_z;

  Vector3d e1 = (Dg-G).normalized();
  Vector3d e2 = Vector3d::UnitZ();
  Vector3d K = e1.cross(e2);
  Vector3d C = point_at_grip_depth(cone_state,config.rnw.desired_grip_depth);

  if ( !_init ) {
    ROS_ERROR_STREAM("[rnw] did not init corridor!");
    return C;
  }

  // make sure they are radiant
  double cur_nutation = cone_state.euler_angles.y;
  double desired_nutation = config.rnw.desired_nutation*deg2rad;
  double theta = desired_nutation - cur_nutation;
  // rotate along K, positive rotation increase nutation
  Vector3d C_prime = rotate_point_along_axis(C,G,K,theta);

  // then sideways

  if ( abs(cone_state.euler_angles.z) > (5 * deg2rad) ) {
    if ( cone_state.euler_angles.z > 0 ){
      dir = -1;
    }
    else {
      dir = 1;
    }

    if ( abs(cone_state.euler_angles.z) > 1.2 ) {
      ROS_ERROR_STREAM("[rnw] amp too large!!!");
      //dir = -dir;
    }

  }

  Eigen::Vector2d C_prime_2d(C_prime.x(), C_prime.y());
  Eigen::Vector2d ptc = Rcw * C_prime_2d + Tcw;
  ptc.y() = dir * corridor_width;
  Eigen::Vector2d C_next_2d = Rwc * ptc + Twc;

  Vector3d C_next(C_next_2d.x(), C_next_2d.y(), C_prime.z() );


  return C_next;
}