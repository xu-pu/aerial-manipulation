#include "rnw_ros/rnw_planner_v2.h"

#include <uav_utils/utils.h>

rnw_planner_v2_t::rnw_planner_v2_t( rnw_config_t const & cfg ) : rnw_config(cfg) {
  ros::NodeHandle nh("~");
  pub_rnw_state = nh.advertise<rnw_msgs::RnwState>("/rnw/state",100);
}

void rnw_planner_v2_t::on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
  latest_cone_state = *msg;
  cone_state_init = true;
}

void rnw_planner_v2_t::spin(){

  control_loop();

  ///// Publish debug/log information

  pub_rnw_state.publish(to_rnw_state());

}

void rnw_planner_v2_t::reset_states(){
  step_count = 0;
  peak_phi_dot = 0;
  peak_phi_dot_history.clear();
  energy_initialized = false;
  step_direction = 1;
  tau = 0;
  e_KE_integral = 0;
  data_log = { 0 };
}

void rnw_planner_v2_t::start_walking(){

  if ( is_walking ) return;

  reset_states();
  is_walking = true;
  walk_idx++;

  /**
   * Setup Heading Direction
   */

  if ( rnw_config.rnw.specify_heading ) {
    desired_heading_direction = rnw_config.rnw.heading;
    ROS_INFO("[rnw_planner] heading direction specified in config");
  }
  else  {
    desired_heading_direction = calc_cone_heading_direction(latest_cone_state);
    ROS_INFO("[rnw_planner] did not specify heading direction, use the current value");
  }
  heading_step_pos = desired_heading_direction;
  heading_step_neg = desired_heading_direction;

}

void rnw_planner_v2_t::stop_walking(){
  if ( is_walking ) {
    is_walking = false;
    reset_states();
  }
}

rnw_msgs::RnwState rnw_planner_v2_t::to_rnw_state() const {
  rnw_msgs::RnwState msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.is_walking = is_walking;
  msg.step_count = step_count;
  msg.walk_count = walk_idx;
  msg.step_direction = static_cast<int8_t>(step_direction);
  msg.cmd_time = cmd.stamp;
  msg.cmd_idx = cmd.seq;
  msg.cmd_setpoint = uav_utils::to_point_msg(cmd.setpoint);

  msg.initialized = energy_initialized;
  msg.tau = (float)tau;
  msg.tau_ff = (float)data_log.tau_ff;
  msg.tau_energy_term = (float)data_log.tau_energy_term;
  msg.tau_steering_term = (float)data_log.tau_steering_term;
  msg.tilt = (float)data_log.tilt;
  msg.e_KE = (float)data_log.e_KE;
  msg.e_KE_integral = (float)e_KE_integral;
  msg.e_psi = (float)data_log.e_psi;
  msg.peak_phi_dot = (float)(peak_phi_dot_history.empty() ? 0. : peak_phi_dot_history.back());

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

  /////////// From this point, r-n-w is normal and in progress

  double ang_vel_thresh = rnw_config.rnw.ang_vel_threshold;
  double min_step_interval = rnw_config.rnw.min_step_interval;

  if ( !energy_initialized ) {
    ang_vel_thresh = rnw_config.rnw.init_ang_vel_threshold;
    min_step_interval = rnw_config.rnw.init_min_step_interval;
  }

//  if ( !peak_phi_dot_history.empty() ) {
//    ang_vel_thresh = uav_utils::clamp<double>(0.5*peak_phi_dot_history.back(),0.1,ang_vel_thresh);
//  }

  // for the energy controller
  peak_phi_dot = std::max(peak_phi_dot,std::abs(latest_cone_state.euler_angles_velocity.z));

  // avoid transient states
  if (ros::Time::now() - cmd.stamp < ros::Duration(min_step_interval) ) {
    return;
  }

  if ( abs(latest_cone_state.euler_angles_velocity.z) > ang_vel_thresh ) {
    return;
  }

  if ( !energy_initialized && peak_phi_dot > rnw_config.rnw.init_threshold ) {
    energy_initialized = true;
    ROS_WARN("[rnw_planner] rnw energy initialized!");
  }

  /**
   * if phi > epsi, make sure rot direction match, otherwise ignore the direction condition
   * this will allow initialization
   */

  bool direction_matched = -1 * step_direction * latest_cone_state.euler_angles.z >= rnw_config.rnw.phi_epsi;

  bool low_energy = std::abs(latest_cone_state.euler_angles.z) < rnw_config.rnw.phi_epsi;

  if ( direction_matched || low_energy ) {
    plan_next_step();
  }

}

void rnw_planner_v2_t::plan_next_step(){

  cmd.stamp = ros::Time::now();

  // adjust nutation first

  Vector3d G = uav_utils::from_point_msg(latest_cone_state.contact_point);
  Vector3d D = uav_utils::from_point_msg(latest_cone_state.disc_center);
  Vector3d Dg = D; Dg.z() = rnw_config.ground_z;

  Vector3d e1 = (Dg-G).normalized();
  Vector3d e2 = Vector3d::UnitZ();
  Vector3d K = e1.cross(e2);
  Vector3d C = uav_utils::from_point_msg(latest_cone_state.tip);

  // make sure they are radiant
  double cur_nutation = latest_cone_state.euler_angles.y;
  double desired_nutation = rnw_config.rnw.desired_nutation*deg2rad;
  double theta = desired_nutation - cur_nutation;
  // rotate along K, positive rotation increase nutation
  Vector3d C_prime = rotate_point_along_axis(C,G,K,theta);

  /**
   * regulate heading direction
   */

  double steering_term = 0;

  if ( rnw_config.rnw.enable_steering && step_count > 2 ) {
    double heading_err = uav_utils::normalize_angle( heading_step_pos + heading_step_neg - 2 * desired_heading_direction ) / 2;
    data_log.e_psi = rad2deg * heading_err;
    steering_term = - rnw_config.rnw.yaw_gain * heading_err;
  }

  data_log.tau_steering_term = rad2deg * steering_term;

  /**
   * regulate step magnitude based on energy
   * negative steps follow the positive steps to ensure symmetry
   */

  if ( step_direction > 0 ) {

    double energy_term = 0;

    double tau_ff = energy_initialized ? rnw_config.rnw.tau : rnw_config.rnw.init_tau;

    if ( rnw_config.rnw.enable_energy_feedback && energy_initialized && !peak_phi_dot_history.empty() ) {
      double des_KE = rnw_config.rnw.specify_energy ? rnw_config.rnw.desired_energy : peak_phi_dot_history.back();
      double e_KE = peak_phi_dot - des_KE;
      e_KE_integral += e_KE;
      energy_term = - (rnw_config.rnw.EKp * e_KE + rnw_config.rnw.EKi * e_KE_integral );
      data_log.e_KE = e_KE;
    }

    data_log.tau_ff = tau_ff;
    data_log.tau_energy_term = rad2deg * energy_term;

    tau = std::max<double>( tau_ff * deg2rad + energy_term, 0. );

  }

  double rot_rad = step_direction * tau + steering_term;

  data_log.tilt = rad2deg * rot_rad;

  Matrix3d rot = Eigen::AngleAxisd(rot_rad,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d v = C_prime - G;
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;

  cmd.setpoint = setpoint_apex;
  cmd.heading = desired_heading_direction;
  cmd.seq++;
  step_count++;

  if ( step_direction > 0 ) {
    heading_step_pos = calc_cone_heading_direction(latest_cone_state);
  }
  else {
    heading_step_neg = calc_cone_heading_direction(latest_cone_state);
  }

  if ( step_direction > 0 ) {
    peak_phi_dot_history.push_back(peak_phi_dot);
  }

  ////////////////////////////////

  step_direction = -step_direction;

  peak_phi_dot = 0;

}