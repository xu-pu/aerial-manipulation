//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/pose_utils.h"
#include <uav_utils/geometry_utils.h>

Vector3d cone_rot2euler( Matrix3d const & R ){

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

Vector3d tip_position_to_uav_position( Vector3d const & tip, rnw_config_t const & config ){
  Vector3d rst = tip;
  rst.z()  = rst.z() - config.insert_below_tip - config.X_tcp_cage.z();
  return rst;
}

vector<Vector3d> gen_wpts_push_topple(
        nav_msgs::Odometry const & uav_odom,
        rnw_msgs::ConeState const & cone_state,
        rnw_config_t const & rnw_config )
{

  vector<Vector3d> waypoints_C;

  Vector3d apex_init = point_at_grip_depth(cone_state, rnw_config.caging.desired_grip_depth);
  Vector3d fulcrum = apex_init; fulcrum.z() = rnw_config.ground_z;
  Vector3d axis = uav_utils::from_quaternion_msg(cone_state.odom.pose.pose.orientation)
          .normalized()
          .toRotationMatrix()
          .col(1)
          .normalized();
  Vector3d backward_dir = -1 * uav_utils::from_quaternion_msg(cone_state.odom.pose.pose.orientation)
          .normalized()
          .toRotationMatrix()
          .col(0)
          .normalized();

  constexpr size_t segments = 5;
  double rad_step = deg2rad * rnw_config.rnw.desired_nutation / segments;
  constexpr double backward_space = 0.1;

  waypoints_C.emplace_back( apex_init + backward_space * backward_dir );
  for ( size_t i=0; i<=segments; i++ ) {
    waypoints_C.push_back(rotate_point_along_axis(apex_init,fulcrum,axis,i*rad_step));
  }

  /// waypoints for gripping point planned, now convert to uav waypoints

  vector<Vector3d> waypoints_uav;
  waypoints_uav.push_back(uav_utils::from_point_msg(uav_odom.pose.pose.position));
  for ( auto const & C : waypoints_C ) {
    waypoints_uav.push_back(tcp2uav(C,uav_odom,rnw_config.flu_T_tcp));
  }

  return waypoints_uav;

}

vector<Vector3d> gen_wpts_insert_topple(rnw_config_t const & rnw_config ){

  vector<Vector3d> wpts;
  wpts.emplace_back(0,0,rnw_config.rnw.hover_above_tip); // above tip
  //wpts.emplace_back(0,0,0); // tip point
  wpts.emplace_back(0,0,-rnw_config.rnw.insertion_depth); // inserted

  Vector3d offset(rnw_config.rnw.topple_init,0,-rnw_config.rnw.insertion_depth);

  constexpr size_t segments = 5;
  double rad_step = rnw_config.rnw.desired_nutation/segments*deg2rad;

  for ( size_t i=0; i<=segments; i++ ) {
    double rad = i*rad_step;
    double forward = sin(rad)*rnw_config.cone.height();
    double downward = (1-cos(rad))*rnw_config.cone.height();
    Vector3d v(forward,0,-downward);
    wpts.emplace_back(offset+v);
  }

  return wpts;

}

void rnw_config_t::load_from_ros( ros::NodeHandle & nh ){

  X_tcp_cage.x() = get_param_default(nh,"/X_tcp_cage/x",0.);
  X_tcp_cage.y() = get_param_default(nh,"/X_tcp_cage/y",0.);
  X_tcp_cage.z() = get_param_default(nh,"/X_tcp_cage/z",0.);

  flu_T_tcp.x() = get_param_default(nh,"/flu_T_tcp/x",0.);
  flu_T_tcp.y() = get_param_default(nh,"/flu_T_tcp/y",0.);
  flu_T_tcp.z() = get_param_default(nh,"/flu_T_tcp/z",0.);

  insert_below_tip = get_param_default(nh,"/insert_below_tip",5.);

  zigzag.step_forward = get_param_default(nh,"/zigzag/step_forward",0.1);
  zigzag.step_sideways = get_param_default(nh,"/zigzag/step_sideways",0.1);
  zigzag.cycles = get_param_default(nh,"/zigzag/cycles",5);
  zigzag.max_vel = get_param_default(nh,"/zigzag/max_vel",1);
  zigzag.max_acc = get_param_default(nh,"/zigzag/max_acc",0.5);

  topple.forward = get_param_default(nh,"/topple/forward",0.2);
  topple.downward = get_param_default(nh,"/topple/downward",0.03);

  cone.radius = get_param_default(nh,"/cone/radius",0.15);

  cone.tip.x() = get_param_default<double>(nh,"/cone/tip/x",0.);
  cone.tip.y() = get_param_default<double>(nh,"/cone/tip/y",0.);
  cone.tip.z() = get_param_default<double>(nh,"/cone/tip/z",0.);

  cone.base_center.x() = get_param_default<double>(nh,"/cone/base_center/x",0.);
  cone.base_center.y() = get_param_default<double>(nh,"/cone/base_center/y",0.);
  cone.base_center.z() = get_param_default<double>(nh,"/cone/base_center/z",0.);

  cone.CoM_x = get_param_default<double>(nh,"/cone/CoM_x",0.);
  cone.CoM_z = get_param_default<double>(nh,"/cone/CoM_z",0.);

  cone.mass = get_param_default<double>(nh,"/cone/mass",0.5);

  ground_z = get_param_default<double>(nh, "/ground_z", 0.);

  rnw.insertion_depth  = get_param_default<double>(nh, "/rnw/insertion_depth", 0.);
  rnw.topple_init      = get_param_default<double>(nh, "/rnw/topple_init", 0.);
  rnw.desired_nutation = get_param_default<double>(nh, "/rnw/desired_nutation", 30.);
  rnw.tau              = get_param_default<double>(nh, "/rnw/tau", 30.);
  rnw.max_vel          = get_param_default<double>(nh, "/rnw/max_vel", 0.5);
  rnw.max_acc          = get_param_default<double>(nh, "/rnw/max_acc", 0.5);
  rnw.rocking_max_vel  = get_param_default<double>(nh, "/rnw/rocking_max_vel", 0.5);
  rnw.rocking_max_acc  = get_param_default<double>(nh, "/rnw/rocking_max_acc", 0.5);
  rnw.hover_above_tip  = get_param_default<double>(nh,"/rnw/hover_above_tip",0.03);
  rnw.adjust_nutation_threshold  = get_param_default<double>(nh,"/rnw/adjust_nutation_threshold",5.);
  rnw.adjust_grip_depth_threshold = get_param_default<double>(nh, "/rnw/adjust_grip_depth_threshold", 0.05);
  rnw.ang_vel_threshold = get_param_default<double>(nh, "/rnw/ang_vel_threshold", 0);
  rnw.min_nutation_deg = get_param_default<double>(nh, "/rnw/min_nutation_deg", 0);
  rnw.yaw_gain = get_param_default<double>(nh, "/rnw/yaw_gain", 0);
  rnw.desired_spin_deg = get_param_default<double>(nh, "/rnw/desired_spin_deg", 30);
  rnw.spin_Kp = get_param_default<double>(nh, "/rnw/spin_Kp", 1);
  rnw.spin_Ki = get_param_default<double>(nh, "/rnw/spin_Ki", 1);
  rnw.lap_ang_vel_deg = get_param_default<double>(nh, "/rnw/lap_ang_vel_deg", 0);
  rnw.lap_start = (size_t)get_param_default<int>(nh, "/rnw/lap_start", 6);
  rnw.enable_steering = get_param_default<bool>(nh, "/rnw/enable_steering", true);
  rnw.enable_energy_feedback = get_param_default<bool>(nh, "/rnw/enable_energy_feedback", true);
  rnw.EKp = get_param_default<double>(nh, "/rnw/EKp", 0);
  rnw.EKi = get_param_default<double>(nh, "/rnw/EKi", 0);
  rnw.EKd = get_param_default<double>(nh, "/rnw/EKd", 0);
  rnw.tau_ff = get_param_default<double>(nh, "/rnw/tau_ff", 0);

  swarm.angle = get_param_default<double>(nh, "/swarm/angle", 90);

  cable.drone1 = get_param_default<double>(nh, "/cable/drone1", 1);
  cable.drone2 = get_param_default<double>(nh, "/cable/drone2", 1);

  caging.desired_grip_depth = get_param_default<double>(nh, "/caging/desired_grip_depth", 0.06);

}

double dist( Vector3d const & A, Vector3d const & B ){
  return (A-B).norm();
}

double line_point_dist_3d( Vector3d const & A, Vector3d const & B, Vector3d const & C ){
  return ((C-A).cross(C-B)).norm()/(A-B).norm();
}

Vector3d tcp2uav( Vector3d const & tcp, nav_msgs::Odometry const & uav_odom, Vector3d const & flu_T_tcp ){
  return tcp - uav_utils::from_quaternion_msg(uav_odom.pose.pose.orientation) * flu_T_tcp;
}

Vector3d point_at_grip_depth( rnw_msgs::ConeState const & cone_state, double grip_depth ){
  Vector3d tip = uav_utils::from_point_msg(cone_state.tip);
  Vector3d base = uav_utils::from_point_msg(cone_state.base);
  Vector3d dir = (base-tip).normalized();
  return tip + grip_depth * dir;
}

Vector3d rotate_point_along_axis( Vector3d const & X, Vector3d const & O, Vector3d const & K, double theta ){
  Vector3d V = X-O;
  Vector3d Vrot = V * cos(theta) + K.cross(V) * sin(theta) + K * K.dot(V) * (1-cos(theta));
  return Vrot+O;
}

bool check_waypoints( vector<Vector3d> & wpts ){

  if ( wpts.size() < 2 ) {
    return false;
  }

  // first check points too close together

  constexpr double dist_epsi = 0.005;

  vector<Vector3d> pts;

  pts.push_back(wpts.front());

  for ( size_t i=1; i<wpts.size(); i++ ) {
    Vector3d pt = wpts.at(i);
    if ( dist(pts.back(),pt) > dist_epsi ) {
      pts.push_back(pt);
    }
    else {
      ROS_ERROR_STREAM("[rnw] there is redundant waypoint, removed!");
    }
  }

  if ( pts.size() < 2 ) {
    return false;
  }

  wpts = add_mid_points(pts);

  return true;

}

vector<Vector3d> add_mid_points( vector<Vector3d> const & src ){

  vector<Vector3d> rst;
  rst.push_back(src.front());

  for ( size_t i=1; i<src.size(); i++ ) {
    Vector3d pt = src.at(i);
    rst.emplace_back((rst.back()+pt)/2);
    rst.push_back(pt);
  }

  return rst;

}

double uav_yaw_from_cone_state( rnw_msgs::ConeState const & cone_state ){
  double cone_yaw = uav_utils::get_yaw_from_quaternion(uav_utils::from_quaternion_msg(cone_state.odom.pose.pose.orientation));
  return uav_utils::normalize_angle(cone_yaw-M_PI);
}

double uav_yaw_from_odom( nav_msgs::Odometry const & odom ){
  return uav_utils::get_yaw_from_quaternion(uav_utils::from_quaternion_msg(odom.pose.pose.orientation));
}

double cone_yaw( rnw_msgs::ConeState const & cone_state ){
  return uav_utils::get_yaw_from_quaternion(uav_utils::from_quaternion_msg(cone_state.odom.pose.pose.orientation));
}

double calc_mid_rad( double r1, double r2 ){
  double r = uav_utils::normalize_angle((r1+r2)/2);
  if ( abs(uav_utils::normalize_angle(r-r1)) > M_PI_2 ) {
    r = uav_utils::normalize_angle(r+M_PI);
  }
  return r;
}

double calc_obj_heading( rnw_msgs::ConeState const & s1, rnw_msgs::ConeState const & s2 ){
  double y1 = cone_yaw(s1);
  double y2 = cone_yaw(s2);
  return calc_mid_rad(y1,y2);
}

double uav_yaw_from_cone_yaw( double cone_yaw ){
  return uav_utils::normalize_angle(cone_yaw-M_PI);
}

quadrotor_msgs::PolynomialTrajectory gen_setpoint_traj(nav_msgs::Odometry const & odom, Vector3d const & setpoint, double duration ){
  return gen_setpoint_traj(odom,setpoint,uav_yaw_from_odom(odom),duration);
}

quadrotor_msgs::PolynomialTrajectory gen_setpoint_traj(nav_msgs::Odometry const & odom, Vector3d const & setpoint, double yaw, double duration ){

  constexpr int order = 5;

  quadrotor_msgs::PolynomialTrajectory msg;

  msg.header.stamp = ros::Time::now();
  //msg.trajectory_id = traj_id;
  msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
  msg.num_order = order;
  msg.num_segment = 1;
  msg.start_yaw = uav_yaw_from_odom(odom);
  msg.final_yaw = yaw;

  // fill the segment

  msg.time.push_back(duration);
  msg.order.push_back(order);

  msg.coef_x.push_back(setpoint.x());
  msg.coef_y.push_back(setpoint.y());
  msg.coef_z.push_back(setpoint.z());

  for ( size_t i=0; i<order; i++ ) {
    msg.coef_x.push_back(0);
    msg.coef_y.push_back(0);
    msg.coef_z.push_back(0);
  }

  return msg;

}

Matrix3d intermediate_rotation( nav_msgs::Odometry const & odom ){
  double yaw_curr = uav_utils::get_yaw_from_quaternion(
          uav_utils::from_quaternion_msg(odom.pose.pose.orientation)
  );
  return uav_utils::rotz(yaw_curr);
}

Vector3d point_in_intermediate_frame( Vector3d const & point, nav_msgs::Odometry const & frame ){
  Matrix3d R = intermediate_rotation(frame);
  Vector3d T = uav_utils::from_point_msg(frame.pose.pose.position);
  return R * point + T;
}

vector<Vector3d> points_in_intermediate_frame( vector<Vector3d> const & points, nav_msgs::Odometry const & frame ){

  Matrix3d R = intermediate_rotation(frame);
  Vector3d T = uav_utils::from_point_msg(frame.pose.pose.position);

  vector<Vector3d> rst;
  for ( Vector3d const & iter : points ) {
    rst.emplace_back(R*iter+T);
  }
  return rst;

}

Vector3d point_at_nutation( rnw_msgs::ConeState const & cone_state, Vector3d const & point, double nutation_rad ){
  Vector3d cur_contact = uav_utils::from_point_msg(cone_state.contact_point);
  Vector3d v = point - cur_contact;
  double len = v.norm();
  Vector3d dir_2d = Vector3d(v.x(),v.y(),0).normalized();
  double nut_comp = M_PI_2 - nutation_rad;
  Vector3d tgt = dir_2d * std::cos(nut_comp) * len;
  tgt.z() = std::sin(nut_comp) * len;
  tgt = tgt + cur_contact;
  return tgt;
}