#include "rnw_ros/cone_interface.h"
#include "rnw_ros/ros_utils.h"
#include "rnw_ros/rnw_utils.h"

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
  Vector3d fulcrum = latest_cone_state.is_point_contact ? contact_point() : uav_utils::from_point_msg(latest_cone_state.base);
  Vector3d axis = rnw_frame().col(1);
  double mag = rad - latest_cone_state.euler_angles.y;
  return rotate_point_along_axis(tip(),fulcrum,axis,mag);
}

Matrix3d cone_interface_t::rnw_frame() const {
  return calc_rnw_body_frame(latest_cone_state);
}