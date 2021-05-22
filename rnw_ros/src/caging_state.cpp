#include "rnw_ros/caging_state.h"

#include "rnw_ros/pose_utils.h"

rnw_msgs::GripState caging_state_t::to_msg() const {
  rnw_msgs::GripState msg;
  msg.header = cone_state.header;
  msg.uav_odom = uav_odom;
  msg.cone_state = cone_state;
  msg.flu_T_tcp = uav_utils::to_point_msg(flu_T_tcp);
  msg.grip_point = uav_utils::to_point_msg(grip_point);
  msg.grip_radius = grip_radius;
  msg.grip_depth = grip_depth;
  msg.grip_valid = grip_valid;
  return msg;
}

caging_state_t::caging_state_t() = default;

caging_state_t::caging_state_t(rnw_msgs::ConeState const & _cone_state, nav_msgs::Odometry const & _uav_odom, Vector3d const & _flu_T_tcp )
        : cone_state(_cone_state), uav_odom(_uav_odom), flu_T_tcp(_flu_T_tcp)
{

  Matrix3d R = odom2R(uav_odom);
  Vector3d T = odom2T(uav_odom);
  Vector3d tcp = R * flu_T_tcp + T;
  Vector3d tip = uav_utils::from_point_msg(cone_state.tip);
  Vector3d base = uav_utils::from_point_msg(cone_state.base);
  Vector3d dir = (base - tip).normalized();

  grip_depth = dir.dot(tcp-tip);
  grip_radius = line_point_dist_3d(tip,base,tcp);;

  double tip_tcp = (tip-tcp).norm();
  double tip_grip = sqrt(square(tip_tcp) - square(grip_radius));

  Vector3d grip1 = tip+tip_grip*dir;
  Vector3d grip2 = tip-tip_grip*dir;

  Vector3d grip = grip2;
  if ( dist(grip1,tcp) < dist(grip2,tcp) ) {
    grip = grip1;
  }

  grip_point = grip;

  grip_valid = grip_depth > 0 && grip_radius < 0.1;

  initialized = true;

}