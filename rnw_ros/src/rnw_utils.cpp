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
