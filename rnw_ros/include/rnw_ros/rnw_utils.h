//
// Created by sheep on 2020/8/6.
//

#ifndef SRC_RNW_UTILS_H
#define SRC_RNW_UTILS_H

#include "rnw_ros/ros_utils.h"
#include "rnw_ros/pose_utils.h"
#include <rnw_ros/ConeState.h>
#include <rnw_ros/RockingCmd.h>
#include <quadrotor_msgs/Float64Stamped.h>
#include <uav_utils/utils.h>

struct rnw_config_t {

    Vector3d X_tip_body;

    Vector3d X_tcp_cage;

    double hover_above_tip;

    double insert_below_tip;

    double ground_z;

    struct {

        double forward = 0;

        double downward = 0;

    } topple;

    struct {

        double step_forward = 0;

        double step_sideways = 0;

        size_t cycles = 0;

        double max_vel = 0;

        double max_acc = 0;

    } zigzag;

    struct {

        double radius;

        double apex;

        double height;

    } cone;

    inline void load_from_ros( ros::NodeHandle & nh ){

      X_tip_body.x() = get_param_default(nh,"X_tip_body/x",0.);
      X_tip_body.y() = get_param_default(nh,"X_tip_body/y",0.);
      X_tip_body.z() = get_param_default(nh,"X_tip_body/z",0.);

      X_tcp_cage.x() = get_param_default(nh,"X_tcp_cage/x",0.);
      X_tcp_cage.y() = get_param_default(nh,"X_tcp_cage/y",0.);
      X_tcp_cage.z() = get_param_default(nh,"X_tcp_cage/z",0.);

      hover_above_tip = get_param_default(nh,"hover_above_tip",5.);
      insert_below_tip = get_param_default(nh,"insert_below_tip",5.);

      zigzag.step_forward = get_param_default(nh,"zigzag/step_forward",0.1);
      zigzag.step_sideways = get_param_default(nh,"zigzag/step_sideways",0.1);
      zigzag.cycles = get_param_default(nh,"zigzag/cycles",5);
      zigzag.max_vel = get_param_default(nh,"zigzag/max_vel",1);
      zigzag.max_acc = get_param_default(nh,"zigzag/max_acc",0.5);

      topple.forward = get_param_default(nh,"topple/forward",0.2);
      topple.downward = get_param_default(nh,"topple/downward",0.03);

      cone.radius = get_param_default(nh,"cone/radius",0.15);
      cone.height = get_param_default(nh,"cone/height",1);
      cone.apex = get_param_default(nh,"cone/apex",0.8);

      ground_z = get_param_default<double>(nh, "ground_z", 0);

    }

};

/**
 * insertion waypoints in the local frame of the cone tip
 * @param hover_above
 * @param insert_below
 * @param head_room
 * @param topple_forward
 * @return
 */
inline vector<Vector3d> gen_topple_waypoints_local(
        double hover_above = 0.05,
        double insert_below = 0.03,
        double topple_forward = 0.2,
        double topple_downward = 0.03,
        double head_room = 0.2 )
{
  vector<Vector3d> wpts;
  wpts.emplace_back(head_room,0,hover_above); // ahead
  wpts.emplace_back(0,0,hover_above); // above tip
  wpts.emplace_back(0,0,0); // tip point
  wpts.emplace_back(0,0,-insert_below); // inserted
  wpts.emplace_back(topple_forward,0,-insert_below-topple_downward); // toppled
  return wpts;
}


/**
 * x-psi-precession
 * y-theta-nutation (tilt)
 * z-phi-spin
 * @param R
 * @return - (x,y,z)
 */
inline Vector3d cone_rot2euler( Matrix3d const & R ){

  double psi=0,theta=0,phi=0;

  auto r00 = R(0,0);
  auto r02 = R(0,2);
  auto r10 = R(1,0);
  auto r12 = R(1,2);
  auto r20 = R(2,0);
  auto r21 = R(2,1);
  auto r22 = R(2,2);

  constexpr double epsi = 0.05;

  theta = atan2(sqrt(r02*r02+r12*r12),r22);

//  if ( r22 == -1 ) {
//    phi = 0;
//    psi = atan2(r00,-r10);
//
//  }
//  else if ( r22 == 1 ){
//    phi = 0;
//    psi = atan2(-r00,r10);
//  }
  if ( abs(theta) < epsi ) {
    phi = 0;
    psi = atan2(-r00,r10);
  }
  else {
    phi = atan2(r21,-r20);
    psi = atan2(-r02,r12);
  }

  return { psi, theta, phi };

}

struct cone_state_estimator_t {

    // object properties

    rnw_config_t rnw_config;

    double diameter() const {
      return rnw_config.cone.radius * 2;
    }

    Vector3d X_base_body(){
      Vector3d rst = rnw_config.X_tip_body;
      rst.z() = rnw_config.X_tip_body.z() - rnw_config.cone.height;
      return rst;
    }

    Vector3d X_center_body(){
      Vector3d rst = X_base_body();
      rst.x() -= rnw_config.cone.radius;
      return rst;
    }

    //////////////////////////////

    ros::Publisher pub_cone_state;

    ros::Publisher pub_odom_dt;

    // estimator state

    nav_msgs::Odometry previous_odom;

    bool init = false;

    // parameters

    static constexpr double min_tilt = 5;

    double odom_timeout = 1;

    bool cut_euler_velocity = false;

    double max_euler_velocity = numeric_limits<double>::max();

    // latest states

    Vector3d latest_euler_angles;

    Vector3d latest_euler_velocity;

    Matrix3d R_markers;

    Vector3d T_markers;

    Vector3d T_tip;

    Vector3d T_base;

    Vector3d T_center;

    bool contact_valid = false;

    Vector3d contact_point;

    inline explicit cone_state_estimator_t( ros::NodeHandle & nh ){
      rnw_config.load_from_ros(nh);
      pub_cone_state = nh.advertise<rnw_ros::ConeState>("cone_state",10);
      pub_odom_dt = nh.advertise<quadrotor_msgs::Float64Stamped>("dt",10);
      cut_euler_velocity = get_param_default(nh,"cut_euler_velocity",false);
      max_euler_velocity = get_param_default(nh,"max_euler_velocity",numeric_limits<double>::max());
    }

    inline void on_odom( nav_msgs::OdometryConstPtr const & msg ){

      if ( !init ) {
        previous_odom = *msg;
        init = true;
        ROS_INFO_STREAM("[Cone] Initialized");
        return;
      }
      else if (msg_time_diff(previous_odom, *msg) > odom_timeout ) {
        previous_odom = *msg;
        ROS_ERROR_STREAM("[Cone] Odom Timeout, re-initialized!");
        return;
      }
      else if (msg_time_diff(previous_odom, *msg) < 0 ) {
        previous_odom = *msg;
        ROS_ERROR_STREAM("[Cone] Message out of order, re-initialized!");
        return;
      }

      update_euler(msg);

      update_frames(msg);

      update_contact_point();

      publish_cone_state(msg);

      previous_odom = *msg;

    }

    inline void publish_cone_state( nav_msgs::OdometryConstPtr const & msg ) const {
      rnw_ros::ConeState msg_cone;
      msg_cone.header.stamp = msg->header.stamp;
      msg_cone.odom = *msg;
      msg_cone.euler_angles = uav_utils::to_vector3_msg(latest_euler_angles);
      msg_cone.euler_angles_velocity = uav_utils::to_vector3_msg(latest_euler_velocity);
      msg_cone.is_point_contact = contact_valid;
      msg_cone.contact_point = uav_utils::to_point_msg(contact_point);
      msg_cone.base = uav_utils::to_point_msg(T_base);
      msg_cone.tip = uav_utils::to_point_msg(T_tip);
      msg_cone.disc_center = uav_utils::to_point_msg(T_center);
      pub_cone_state.publish(msg_cone);
    }

    inline void update_euler(  nav_msgs::OdometryConstPtr const & msg ){

      auto pre = previous_odom;
      auto cur = *msg;

      double dt = msg_time_diff(pre,cur);

      quadrotor_msgs::Float64Stamped msg_dt;
      msg_dt.header.stamp = msg->header.stamp;
      msg_dt.value = dt;
      pub_odom_dt.publish(msg_dt);

      Vector3d euler_cur = cone_rot2euler(odom2R(cur));
      Vector3d euler_pre = cone_rot2euler(odom2R(pre));

      Vector3d euler_diff = euler_cur - euler_pre;
      //ROS_INFO_STREAM("before " << euler_diff.transpose());
      euler_diff(0) = uav_utils::normalize_angle(euler_diff(0));
      euler_diff(1) = uav_utils::normalize_angle(euler_diff(1));
      euler_diff(2) = uav_utils::normalize_angle(euler_diff(2));
      //ROS_INFO_STREAM("after: " << euler_diff.transpose());

      Vector3d euler_vel = euler_diff/dt;

      if ( cut_euler_velocity ) {
        euler_vel(0) = min(euler_vel(0),max_euler_velocity);
        euler_vel(1) = min(euler_vel(1),max_euler_velocity);
        euler_vel(2) = min(euler_vel(2),max_euler_velocity);
        euler_vel(0) = max(euler_vel(0),-max_euler_velocity);
        euler_vel(1) = max(euler_vel(1),-max_euler_velocity);
        euler_vel(2) = max(euler_vel(2),-max_euler_velocity);
      }

      latest_euler_angles = euler_cur;
      latest_euler_velocity = euler_vel;

    }

    inline void update_frames( nav_msgs::OdometryConstPtr const & msg ){
      R_markers = odom2R(msg);
      T_markers = odom2T(msg);
      T_tip = R_markers * rnw_config.X_tip_body + T_markers;
      T_base = R_markers * X_base_body() + T_markers;
      T_center = R_markers * X_center_body() + T_markers;
    }

    /**
     * 3D plane of object's base, A(x-x0)+B(y-y0)+C(z-z0)=0
     * where (A,B,C) is the normal vector, and (x0,y0,z0) is center of the disc
     * The plane intersect with z=ground_z, get a 2d line A'x+B'y+C'=0, normal vector (A',B')
     * (x0,y0) is disc center on z=ground_z plane
     * contact point = (x0,y0) + lambda * (A',B') which lies on A'x+B'y+C'=0
     */
    inline void update_contact_point(){

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

      if ( ratio > 1.1 ) {
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

};

#endif //SRC_RNW_UTILS_H
