#include "uart_odom/drone_swarm_payload.h"

using namespace uwb_comm;

drone_swarm_payload_t::drone_swarm_payload_t(ros::NodeHandle & _nh) : nh(_nh) {}

void drone_swarm_payload_t::on_cmd(const quadrotor_msgs::PositionCommandConstPtr & msg) {
  latest_cmd = *msg;
}

void drone_swarm_payload_t::on_odom(const nav_msgs::OdometryConstPtr & msg) {
  latest_odom = *msg;
}

void drone_swarm_payload_t::init_as_slave() {}

void drone_swarm_payload_t::init_as_master() {}

void drone_swarm_payload_t::decode(const char *buffer) {}

void drone_swarm_payload_t::encode(char *buffer) {}

int drone_swarm_payload_t::data_length() {}