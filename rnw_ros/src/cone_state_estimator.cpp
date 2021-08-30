//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/cone_state_estimator.h"
#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>
#include "rnw_ros/pose_utils.h"
#include "rnw_msgs/ConeState.h"

Vector3d cone_state_estimator_t::X_base_body() const {
  Vector3d rst = rnw_config.cone.tip;
  rst.z() = rnw_config.cone.base_center.z();
  return rst;
}

cone_state_estimator_t::cone_state_estimator_t( ros::NodeHandle & nh ) {

  rnw_config.load_from_ros(nh);

  pub_cone_state = nh.advertise<rnw_msgs::ConeState>("/cone/state",10);

  pub_odom_dt = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/dt",10);

  sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "/cone/odom",
          10,
          &cone_state_estimator_t::on_odom,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  lpf_ang_vel_x.T = get_param_default<double>(nh,"lpf_ang_vel_x",0.1);
  lpf_ang_vel_y.T = get_param_default<double>(nh,"lpf_ang_vel_y",0.1);
  lpf_ang_vel_z.T = get_param_default<double>(nh,"lpf_ang_vel_z",0.1);

}

void cone_state_estimator_t::on_odom( nav_msgs::OdometryConstPtr const & msg ){

  if ( !init ) {
    latest_odom = *msg;
    update_euler_angles();
    init = true;
    ROS_INFO_STREAM("[Cone] Initialized");
    return;
  }
  else if (msg_time_diff(latest_odom, *msg) > odom_timeout ) {
    latest_odom = *msg;
    update_euler_angles();
    ROS_ERROR_STREAM("[Cone] Odom Timeout, re-initialized!");
    return;
  }
  else if (msg_time_diff(latest_odom, *msg) < 0 ) {
    latest_odom = *msg;
    update_euler_angles();
    ROS_ERROR_STREAM("[Cone] Message out of order, re-initialized!");
    return;
  }

  update_euler_velocity(msg);

  update_body_points(msg);

  update_contact_point();

  publish_cone_state(msg);

  latest_odom = *msg;

}

void cone_state_estimator_t::publish_cone_state( nav_msgs::OdometryConstPtr const & msg ) const {
  rnw_msgs::ConeState msg_cone;
  msg_cone.header.stamp = msg->header.stamp;
  msg_cone.odom = *msg;
  msg_cone.euler_angles = uav_utils::to_vector3_msg(latest_euler_angles);
  msg_cone.euler_angles_velocity = uav_utils::to_vector3_msg(latest_euler_velocity);
  msg_cone.is_point_contact = contact_valid;
  msg_cone.contact_point = uav_utils::to_point_msg(contact_point);
  msg_cone.base = uav_utils::to_point_msg(T_base);
  msg_cone.tip = uav_utils::to_point_msg(T_tip);
  msg_cone.disc_center = uav_utils::to_point_msg(T_center);
  msg_cone.radius = rnw_config.cone.radius;
  msg_cone.height = rnw_config.cone.tip.z() - rnw_config.cone.base_center.z();

  pub_cone_state.publish(msg_cone);
}

void cone_state_estimator_t::update_euler_angles(){
  latest_euler_angles = cone_rot2euler(odom2R(latest_odom));
}

void cone_state_estimator_t::update_euler_velocity(nav_msgs::OdometryConstPtr const & msg ){

  auto pre = latest_odom;
  auto cur = *msg;
  latest_odom = *msg;

  double dt = msg_time_diff(pre,cur);

  quadrotor_msgs::Float64Stamped msg_dt;
  msg_dt.header.stamp = msg->header.stamp;
  msg_dt.value = dt;
  pub_odom_dt.publish(msg_dt);

  Vector3d euler_pre = latest_euler_angles;
  update_euler_angles();
  Vector3d euler_cur = latest_euler_angles;

  Vector3d euler_diff = euler_cur - euler_pre;
  //ROS_INFO_STREAM("before " << euler_diff.transpose());
  euler_diff(0) = uav_utils::normalize_angle(euler_diff(0));
  euler_diff(1) = uav_utils::normalize_angle(euler_diff(1));
  euler_diff(2) = uav_utils::normalize_angle(euler_diff(2));
  //ROS_INFO_STREAM("after: " << euler_diff.transpose());

  Vector3d euler_vel = euler_diff/dt;
  euler_vel.x() = lpf_ang_vel_x.filter(euler_vel.x());
  euler_vel.y() = lpf_ang_vel_y.filter(euler_vel.y());
  euler_vel.z() = lpf_ang_vel_z.filter(euler_vel.z());

  latest_euler_angles = euler_cur;
  latest_euler_velocity = euler_vel;

}

void cone_state_estimator_t::update_body_points(nav_msgs::OdometryConstPtr const & msg ){
  R_markers = odom2R(msg);
  T_markers = odom2T(msg);
  T_tip = R_markers * rnw_config.cone.tip + T_markers;
  T_base = R_markers * X_base_body() + T_markers;
  T_center = R_markers * rnw_config.cone.base_center + T_markers;
}

void cone_state_estimator_t::update_contact_point(){

  double tilt_x = abs(asin(R_markers(2,0)));
  double tilt_x_deg = tilt_x / M_PI * 180;
  if ( tilt_x_deg < min_tilt ) {
    ROS_WARN_STREAM("not point contact, " << tilt_x_deg );
    contact_valid = false;
    return;
  }

  double x0 = T_center.x();
  double y0 = T_center.y();
  double z0 = T_center.z();

  double zg = rnw_config.ground_z;

  Vector3d n = R_markers.col(2);

  double A = n.x();
  double B = n.y();
  double C = n.z()*(zg-z0) - A*x0 - B*y0;

  double dist_2d = abs(A*x0 + B*y0 + C) / sqrt(A*A + B*B);
  double dist = sqrt(dist_2d*dist_2d + z0*z0);

  double ratio = dist/rnw_config.cone.radius;

  if ( ratio > 1.5 ) {
    // lifted off the ground
    ROS_WARN_STREAM("lifted off the ground");
    contact_valid = false;
    return;
  }

  double lambda = - ( A*x0 + B*y0 + C ) / ( A*A + B*B );

  Vector3d pt = { x0 + lambda*A, y0 + lambda*B, zg };

  contact_point = pt;

  contact_valid = true;

}
