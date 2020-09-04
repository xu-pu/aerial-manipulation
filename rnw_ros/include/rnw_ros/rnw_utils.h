//
// Created by sheep on 2020/8/6.
//

#ifndef SRC_RNW_UTILS_H
#define SRC_RNW_UTILS_H

#include <deque>

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

        Vector3d base_center;

        Vector3d tip;

    } cone;

    struct {

        double insertion_depth;

        double topple_init;

        double desired_nutation;

        double tau;

    } rnw;

    inline void load_from_ros( ros::NodeHandle & nh ){

      X_tip_body.x() = get_param_default(nh,"/X_tip_body/x",0.);
      X_tip_body.y() = get_param_default(nh,"/X_tip_body/y",0.);
      X_tip_body.z() = get_param_default(nh,"/X_tip_body/z",0.);

      X_tcp_cage.x() = get_param_default(nh,"/X_tcp_cage/x",0.);
      X_tcp_cage.y() = get_param_default(nh,"/X_tcp_cage/y",0.);
      X_tcp_cage.z() = get_param_default(nh,"/X_tcp_cage/z",0.);

      hover_above_tip = get_param_default(nh,"/hover_above_tip",5.);
      insert_below_tip = get_param_default(nh,"/insert_below_tip",5.);

      zigzag.step_forward = get_param_default(nh,"/zigzag/step_forward",0.1);
      zigzag.step_sideways = get_param_default(nh,"/zigzag/step_sideways",0.1);
      zigzag.cycles = get_param_default(nh,"/zigzag/cycles",5);
      zigzag.max_vel = get_param_default(nh,"/zigzag/max_vel",1);
      zigzag.max_acc = get_param_default(nh,"/zigzag/max_acc",0.5);

      topple.forward = get_param_default(nh,"/topple/forward",0.2);
      topple.downward = get_param_default(nh,"/topple/downward",0.03);

      cone.radius = get_param_default(nh,"/cone/radius",0.15);
      cone.height = get_param_default(nh,"/cone/height",1);
      cone.apex = get_param_default(nh,"/cone/apex",0.8);

      cone.tip.x() = get_param_default<double>(nh,"/cone/tip/x",0.);
      cone.tip.y() = get_param_default<double>(nh,"/cone/tip/y",0.);
      cone.tip.z() = get_param_default<double>(nh,"/cone/tip/z",0.);

      cone.base_center.x() = get_param_default<double>(nh,"/cone/base_center/x",0.);
      cone.base_center.y() = get_param_default<double>(nh,"/cone/base_center/y",0.);
      cone.base_center.z() = get_param_default<double>(nh,"/cone/base_center/z",0.);

      ground_z = get_param_default<double>(nh, "/ground_z", 0.);

      rnw.insertion_depth  = get_param_default<double>(nh, "/rnw/insertion_depth", 0.);
      rnw.topple_init      = get_param_default<double>(nh, "/rnw/topple_init", 0.);
      rnw.desired_nutation = get_param_default<double>(nh, "/rnw/desired_nutation", 30.);
      rnw.tau              = get_param_default<double>(nh, "/rnw/tau", 30.);

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
        rnw_config_t const & rnw_config,
        double hover_above = 0.05,
        double insert_below = 0.03,
        double topple_forward = 0.2,
        double topple_downward = 0.03,
        double head_room = 0.2 )
{
  vector<Vector3d> wpts;
//  wpts.emplace_back(head_room,0,hover_above); // ahead
  wpts.emplace_back(0,0,hover_above); // above tip
  wpts.emplace_back(0,0,0); // tip point
  wpts.emplace_back(0,0,-insert_below); // inserted
  //wpts.emplace_back(0.05,0,-insert_below); // inserted

  double topple_angle = 30;
  constexpr double deg2rad = M_PI/180.;
  double object_height = 1;
  Vector3d offset(0.05,0,-insert_below);
  size_t segments = 5;
  for ( size_t i=0; i<=segments; i++ ) {
    double rad = i*topple_angle/segments*deg2rad;
    double forward = sin(rad)*object_height;
    double downward = (1-cos(rad))*object_height;
    Vector3d v(forward,0,-downward);
    wpts.push_back(offset+v);
  }

  //wpts.emplace_back(topple_forward,0,-insert_below-topple_downward); // toppled
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

inline Vector3d tip_position_to_uav_position( Vector3d const & tip, rnw_config_t const & config ){
  Vector3d rst = tip;
  rst.z()  = rst.z() - config.insert_below_tip - config.X_tcp_cage.z();
  return rst;
}

template<typename T, size_t window_size>
struct median_filter_t {

    std::deque<T> values;

    median_filter_t() = default;

    T update( T val ){

      values.push_back(val);

      if ( values.size() > window_size ) {
        values.pop_front();
      }

      std::deque<T> sorted(values);
      std::sort(sorted.begin(),sorted.end());

      T output = sorted.at(sorted.size()/2);

      //ROS_INFO_STREAM("[median filter] input=" << val << ", output=" << output );

      return output;

    }

};

template<typename T, size_t window_size>
struct average_filter_t {

    std::deque<T> values;

    average_filter_t() = default;

    T update( T val ){

      values.push_back(val);

      if ( values.size() > window_size ) {
        values.pop_front();
      }

      T sum = 0;

      for ( auto v : values ) { sum+=v; }

      return sum/values.size();

    }

};

struct cone_state_estimator_t {

    // object properties

    rnw_config_t rnw_config;

    double diameter() const {
      return rnw_config.cone.radius * 2;
    }

    Vector3d X_base_body(){
      Vector3d rst = rnw_config.cone.tip;
      rst.z() = rnw_config.cone.tip.z() - rnw_config.cone.height;
      return rst;
    }

    Vector3d X_center_body(){
      return rnw_config.cone.base_center;
    }

    //////////////////////////////

    ros::Publisher pub_cone_state;

    ros::Publisher pub_odom_dt;

    // estimator state

    nav_msgs::Odometry latest_odom;

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

    // filters

    median_filter_t<double,50> ang_vel_z_filter;

    average_filter_t<double,50> ang_y_filter;
    average_filter_t<double,50> ang_z_filter;

    inline explicit cone_state_estimator_t( ros::NodeHandle & nh ) {
      rnw_config.load_from_ros(nh);
      pub_cone_state = nh.advertise<rnw_ros::ConeState>("/rnw/cone_state",10);
      pub_odom_dt = nh.advertise<quadrotor_msgs::Float64Stamped>("dt",10);
      cut_euler_velocity = get_param_default(nh,"cut_euler_velocity",false);
      max_euler_velocity = get_param_default(nh,"max_euler_velocity",numeric_limits<double>::max());
    }

    inline void on_odom( nav_msgs::OdometryConstPtr const & msg ){

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

      update_euler(msg);

      update_frames(msg);

      update_contact_point();

      publish_cone_state(msg);

      latest_odom = *msg;

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

    inline void update_euler_angles(){
      latest_euler_angles = cone_rot2euler(odom2R(latest_odom));
      latest_euler_angles.y() = ang_y_filter.update(latest_euler_angles.y());
      latest_euler_angles.z() = ang_z_filter.update(latest_euler_angles.z());
    }

    inline void update_euler(  nav_msgs::OdometryConstPtr const & msg ){

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

      if ( cut_euler_velocity ) {
        euler_vel(0) = min(euler_vel(0),max_euler_velocity);
        euler_vel(1) = min(euler_vel(1),max_euler_velocity);
        euler_vel(2) = min(euler_vel(2),max_euler_velocity);
        euler_vel(0) = max(euler_vel(0),-max_euler_velocity);
        euler_vel(1) = max(euler_vel(1),-max_euler_velocity);
        euler_vel(2) = max(euler_vel(2),-max_euler_velocity);
      }

      euler_vel.z() = ang_vel_z_filter.update(euler_vel.z());
      //euler_vel = ang_vel_filter.smooth(euler_vel);

      latest_euler_angles = euler_cur;
      latest_euler_velocity = euler_vel;

    }

    inline void update_frames( nav_msgs::OdometryConstPtr const & msg ){
      R_markers = odom2R(msg);
      T_markers = odom2T(msg);
      T_tip = R_markers * rnw_config.cone.tip + T_markers;
      T_base = R_markers * X_base_body() + T_markers;
      T_center = R_markers * X_center_body() + T_markers;
      ROS_INFO_STREAM("" << X_center_body().transpose());
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

struct rnw_planner_t {

    // interface to other modules

    void start_planning_cmd(){
      if ( fsm == cone_fsm_e::idle ) {
        ROS_ERROR_STREAM("[rnw] Can't start planning when object is idle!");
        plan_cmd = false;
      }
      else {
        ROS_INFO_STREAM("[rnw] Start planning rocking commands");
        plan_cmd = true;
      }
    }

    void stop_planning_cmd(){
      ROS_INFO_STREAM("[rnw] Stop planning rocking commands");
      plan_cmd = false;
    }

    /**
     * Call when the current command is complete
     */
    void cmd_ack(){
      cmd_pending = false;
    }

    /**
     * Check is there command to execute
     */
    bool has_pending_cmd() const {
      return cmd_pending;
    }

    Vector3d next_position() const {
      return uav_utils::from_point_msg(latest_cmd.tip_setpoint);
    }

    /**
     * Call on every new ConeState message
     * @param msg
     */
    void on_cone_state( rnw_ros::ConeStateConstPtr const & msg ){

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

    ///////////////////////////////

    static constexpr double deg2rad = M_PI/180.;

    static constexpr double min_tilt = 10 * deg2rad;

    double ang_vel_threshold = 0.5;

    enum class cone_fsm_e {
        idle, qstatic, rocking
    };

    ros::Publisher pub_rocking_cmd;

    cone_fsm_e fsm;

    rnw_ros::ConeState latest_cone_state;

    // planning

    double rot_dir = -1;

    double rot_amp_deg = 30;

    size_t step_count = 0;

    // rocking command

    bool plan_cmd = false;

    rnw_ros::RockingCmd latest_cmd;

    bool cmd_pending = false;

    size_t cmd_idx = 0;

    explicit rnw_planner_t( ros::NodeHandle & nh ){
      pub_rocking_cmd = nh.advertise<rnw_ros::RockingCmd>("/rnw/rocking_cmd",10);
    }

    void plan_next_position(){

      if ( !plan_cmd ) {
        return;
      }

      if ( cmd_pending ) {
        latest_cmd.header.stamp = ros::Time::now();
        pub_rocking_cmd.publish(latest_cmd);
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

    void update_state( rnw_ros::ConeState const & msg ){

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

    void state_transition( cone_fsm_e from, cone_fsm_e to ){

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

    // debug

    void on_debug_trigger( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw] Got debug trigger");
      if ( !plan_cmd ) {
        start_planning_cmd();
      }
      else {
        cmd_ack();
      }
    }

};

#endif //SRC_RNW_UTILS_H
