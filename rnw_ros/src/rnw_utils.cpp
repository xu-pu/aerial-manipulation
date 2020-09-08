//
// Created by sheep on 2020/9/8.
//
#include "rnw_ros/rnw_utils.h"

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

vector<Vector3d> gen_topple_waypoints_local( rnw_config_t const & rnw_config ){

  vector<Vector3d> wpts;
  wpts.emplace_back(0,0,rnw_config.rnw.hover_above_tip); // above tip
  //wpts.emplace_back(0,0,0); // tip point
  wpts.emplace_back(0,0,-rnw_config.rnw.insertion_depth); // inserted

  Vector3d offset(rnw_config.rnw.topple_init,0,-rnw_config.rnw.insertion_depth);

  constexpr double deg2rad = M_PI/180.;
  constexpr size_t segments = 5;
  double rad_step = rnw_config.rnw.desired_nutation/segments*deg2rad;

  for ( size_t i=0; i<=segments; i++ ) {
    double rad = i*rad_step;
    double forward = sin(rad)*rnw_config.cone.height;
    double downward = (1-cos(rad))*rnw_config.cone.height;
    Vector3d v(forward,0,-downward);
    wpts.emplace_back(offset+v);
  }

  return wpts;

}

void rnw_config_t::load_from_ros( ros::NodeHandle & nh ){

  X_tip_body.x() = get_param_default(nh,"/X_tip_body/x",0.);
  X_tip_body.y() = get_param_default(nh,"/X_tip_body/y",0.);
  X_tip_body.z() = get_param_default(nh,"/X_tip_body/z",0.);

  X_tcp_cage.x() = get_param_default(nh,"/X_tcp_cage/x",0.);
  X_tcp_cage.y() = get_param_default(nh,"/X_tcp_cage/y",0.);
  X_tcp_cage.z() = get_param_default(nh,"/X_tcp_cage/z",0.);

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
  rnw.max_vel          = get_param_default<double>(nh, "/rnw/max_vel", 0.5);
  rnw.max_acc          = get_param_default<double>(nh, "/rnw/max_acc", 0.5);
  rnw.rocking_max_vel  = get_param_default<double>(nh, "/rnw/rocking_max_vel", 0.5);
  rnw.rocking_max_acc  = get_param_default<double>(nh, "/rnw/rocking_max_acc", 0.5);
  rnw.hover_above_tip  = get_param_default<double>(nh,"/rnw/hover_above_tip",0.03);
}
