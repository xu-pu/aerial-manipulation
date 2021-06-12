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

  ///// Publish debug/log information

  pub_rnw_state.publish(to_rnw_state());

}

void rnw_planner_v2_t::start_walking(){
  if ( !is_walking ) {
    precession_regulator.start(latest_cone_state);
    is_walking = true;
    walk_idx++;
    step_count = 0;
    peak_phi_dot = 0;
    peak_phi_dot_history.clear();
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

  /**
   * if phi > epsi, make sure rot direction match, otherwise ignore the direction condition
   * this will allow initialization
   */

  double epsi_phi = 10 * deg2rad;

  bool direction_matched = -1 * step_direction * latest_cone_state.euler_angles.z > epsi_phi;

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

  double rot_rad = step_direction * (rnw_config.rnw.tau * deg2rad ) + steering_term;

  Matrix3d rot = Eigen::AngleAxisd(rot_rad,Vector3d::UnitZ()).toRotationMatrix();
  Vector3d v = C_prime - G;
  Vector3d next_v = rot * v;
  Vector3d setpoint_apex = G + next_v;

  cmd.setpoint = setpoint_apex;
  cmd.heading = precession_regulator.desired_heading;
  cmd.seq++;
  step_count++;

  precession_regulator.step(latest_cone_state);

  step_direction = -step_direction;

  peak_phi_dot_history.push_back(peak_phi_dot);
  peak_phi_dot = 0;

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