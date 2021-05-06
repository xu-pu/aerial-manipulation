#include "uwb_transceiver/rnw_payload.h"

#include "uwb_transceiver/mini_odom.h"

using mini_odom_t = Mini_odom<float,int>;

using namespace uwb_comm;

rnw_payload_t::rnw_payload_t( ros::NodeHandle & _nh ) : nh(_nh) {}

void rnw_payload_t::init_as_master(){
  sub_odom_uav = nh.subscribe(
          "odom_in",
          1,
          &rnw_payload_t::on_odom_uav,
          this,
          ros::TransportHints().tcpNoDelay()
  );
}

void rnw_payload_t::on_odom_uav(const nav_msgs::OdometryConstPtr & msg) {
  init = true;
  latest_msg = *msg;
}

void rnw_payload_t::init_as_slave() {
  pub_odom_uav = nh.advertise< nav_msgs::Odometry >( "out_odom_uav", 100 );
  pub_odom_cone = nh.advertise< nav_msgs::Odometry >( "out_odom_cone", 100 );
}

int rnw_payload_t::data_length_master() {
  return sizeof(mini_odom_t) * 2;
}

void rnw_payload_t::slave_decode(const char *buffer) {
  mini_odom_t m_read_mini_odom_uav, m_read_mini_odom_cone;
  memcpy(&m_read_mini_odom_uav, buffer, sizeof(mini_odom_t));
  memcpy(&m_read_mini_odom_cone, buffer+sizeof(mini_odom_t), sizeof(mini_odom_t));
  nav_msgs::Odometry odom_uav, odom_cone;
  miniodom_to_odom(m_read_mini_odom_uav, odom_uav);
  miniodom_to_odom(m_read_mini_odom_cone, odom_cone);
  odom_uav.header.frame_id = "world";
  odom_cone.header.frame_id = "world";
  pub_odom_uav.publish(odom_uav);
  pub_odom_cone.publish(odom_cone);
}

bool rnw_payload_t::master_encode(char *buffer) {
  mini_odom_t m_mini_odom_uav, m_mini_odom_cone;
  odom_to_miniodom<float, int>(latest_msg, m_mini_odom_uav );
  odom_to_miniodom<float, int>( latest_msg, m_mini_odom_cone );
  memcpy(buffer, (char*)&m_mini_odom_uav, sizeof(mini_odom_t));
  memcpy(buffer + sizeof(mini_odom_t), (char*)&m_mini_odom_cone, sizeof(mini_odom_t));
  return init;
}