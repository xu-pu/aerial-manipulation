#include "rnw_ros/rnw_planner_v2.h"

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

void rnw_planner_v2_t::start_walking(){
  if ( !is_walking ) {
    is_walking = true;
    walk_idx++;
    step_count = 0;
    peak_phi_dot = 0;
    peak_phi_dot_history.clear();
    energy_initialized = false;
    latest_tau_rad = 0;
    step_direction = 1;
    latest_tau_rad = 0;
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
}

void rnw_planner_v2_t::stop_walking(){
  if ( is_walking ) {
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
  msg.walk_count = walk_idx;
  msg.step_direction = static_cast<int8_t>(step_direction);
  msg.cmd_time = cmd.stamp;
  msg.cmd_idx = cmd.seq;
  msg.cmd_setpoint = uav_utils::to_point_msg(cmd.setpoint);
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

  // for the energy controller
  peak_phi_dot = std::max(peak_phi_dot,std::abs(latest_cone_state.euler_angles_velocity.z));

  // avoid transient states
  if (ros::Time::now() - cmd.stamp < ros::Duration(rnw_config.rnw.min_step_interval) ) {
    return;
  }

  if ( abs(latest_cone_state.euler_angles_velocity.z) > rnw_config.rnw.ang_vel_threshold
       || abs(latest_cone_state.euler_angles_velocity.x) > rnw_config.rnw.ang_vel_threshold ) {
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

  double epsi_phi = 10 * deg2rad;

  bool direction_matched = -1 * step_direction * latest_cone_state.euler_angles.z >= epsi_phi;

  bool low_energy = std::abs(latest_cone_state.euler_angles.z) < epsi_phi;

  if ( direction_matched || low_energy ) {
    plan_cmd_walk();
  }

}

void rnw_planner_v2_t::plan_cmd_walk(){

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

  // left-right step

  double steering_term = 0;
  if ( rnw_config.rnw.enable_steering && step_count > 2 ) {
    double heading_err = uav_utils::normalize_angle( heading_step_pos + heading_step_neg - 2 * desired_heading_direction ) / 2;
    steering_term = - rnw_config.rnw.yaw_gain * heading_err;
    ROS_INFO("[rnw_planner] deviation = %f deg, steering_term = %f deg",rad2deg*heading_err,rad2deg*steering_term);
  }

  /**
   * regulate step magnitude based on energy
   * negative steps follow the positive steps to ensure symmetry
   */

  if ( step_direction > 0 ) {

    double energy_term = 0;

    if ( rnw_config.rnw.enable_energy_feedback && energy_initialized && !peak_phi_dot_history.empty() ) {
      energy_term = rnw_config.rnw.EKp * ( peak_phi_dot_history.back() - peak_phi_dot );
      ROS_INFO("[rnw_planner] energy_term = %f degrees", rad2deg*energy_term);
    }

    latest_tau_rad = rnw_config.rnw.tau * deg2rad + energy_term;

  }

  double rot_rad = step_direction * latest_tau_rad + steering_term;

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

  ////////////////////////////////

  step_direction = -step_direction;

  peak_phi_dot_history.push_back(peak_phi_dot);
  peak_phi_dot = 0;

}