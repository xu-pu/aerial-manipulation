#include "rnw_ros/cone_interface.h"

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