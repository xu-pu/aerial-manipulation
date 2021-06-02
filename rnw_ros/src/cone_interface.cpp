#include "rnw_ros/cone_interface.h"
#include "rnw_ros/ros_utils.h"

cone_interface_t::cone_interface_t() {

  ros::NodeHandle nh("~");

  sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
          "/cone/state",
          1,
          &cone_interface_t::on_cone_state,
          this,
          ros::TransportHints().tcpNoDelay()
  );

}

void cone_interface_t::on_cone_state(const rnw_msgs::ConeStateConstPtr &msg) {
  latest_cone_state = *msg;
}

bool cone_interface_t::odom_in_time() const {
  return message_in_time(latest_cone_state,msg_timeout);
}

Vector3d cone_interface_t::tip() const {
  return uav_utils::from_point_msg(latest_cone_state.tip);
}

Vector3d cone_interface_t::contact_point() const {
  return uav_utils::from_point_msg(latest_cone_state.contact_point);
}

Vector3d cone_interface_t::tip_at_rest() const {
  return tip_at_nutation(M_PI_2);
}

Vector3d cone_interface_t::tip_at_nutation(double rad) const {
  Vector3d cur_tip = tip();
  Vector3d cur_contact = contact_point();
  Vector3d v = cur_tip - cur_contact;
  double len = v.norm();
  Vector3d dir_2d = Vector3d(v.x(),v.y(),0).normalized();
  double nut_comp = M_PI_2 - rad;
  Vector3d tip = dir_2d * std::cos(nut_comp) * len;
  tip.z() = std::sin(nut_comp) * len;
  return tip + cur_contact;
}