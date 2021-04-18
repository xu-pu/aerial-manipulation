#include "uart_odom/drone_swarm_payload.h"

#include "mini_odom.h"

using namespace uwb_comm;

template<typename T>
struct pos_cmd_data_t {

    uint32_t seq;

    T pos_x;
    T pos_y;
    T pos_z;

    T vel_x;
    T vel_y;
    T vel_z;

    T acc_x;
    T acc_y;
    T acc_z;

    T yaw;
    T yaw_dot;

};


template<typename T>
pos_cmd_data_t<T> encode_pos_cmd( quadrotor_msgs::PositionCommand const & msg ){

  pos_cmd_data_t<T> dat;

  dat.seq = msg.header.seq;

  dat.pos_x = (T)msg.position.x;
  dat.pos_y = (T)msg.position.y;
  dat.pos_z = (T)msg.position.z;

  dat.vel_x = (T)msg.velocity.x;
  dat.vel_y = (T)msg.velocity.y;
  dat.vel_z = (T)msg.velocity.z;

  dat.acc_x = (T)msg.acceleration.x;
  dat.acc_y = (T)msg.acceleration.y;
  dat.acc_z = (T)msg.acceleration.z;

  dat.yaw = (T)msg.yaw;
  dat.yaw_dot = (T)msg.yaw_dot;

  return dat;

}

template<typename T>
quadrotor_msgs::PositionCommand decode_pos_cmd( pos_cmd_data_t<T> const & dat ){

  quadrotor_msgs::PositionCommand msg;

  msg.header.seq = dat.seq;

  msg.position.x = dat.pos_x;
  msg.position.y = dat.pos_y;
  msg.position.z = dat.pos_z;

  msg.velocity.x = dat.vel_x;
  msg.velocity.y = dat.vel_y;
  msg.velocity.z = dat.vel_z;

  msg.acceleration.x = dat.acc_x;
  msg.acceleration.y = dat.acc_y;
  msg.acceleration.z = dat.acc_z;

  msg.yaw = dat.yaw;
  msg.yaw_dot = dat.yaw_dot;

  return msg;

}

using pos_cmd_data_f32_t = pos_cmd_data_t<float>;
using mini_odom_f32_t = Mini_odom<float,int>;

drone_swarm_payload_t::drone_swarm_payload_t(ros::NodeHandle & _nh) : nh(_nh) {}

void drone_swarm_payload_t::on_cmd(const quadrotor_msgs::PositionCommandConstPtr & msg) {
  latest_cmd = *msg;
}

void drone_swarm_payload_t::on_odom(const nav_msgs::OdometryConstPtr & msg) {
  latest_odom = *msg;
}

void drone_swarm_payload_t::init_as_slave() {
  pub_odom = nh.advertise<nav_msgs::Odometry>("odom",100);
  pub_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd",100);
}

void drone_swarm_payload_t::init_as_master() {
  sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "odom",
          1,
          &drone_swarm_payload_t::on_odom,
          this,
          ros::TransportHints().tcpNoDelay()
  );
  sub_cmd = nh.subscribe<quadrotor_msgs::PositionCommand>(
          "position_cmd",
          1,
          &drone_swarm_payload_t::on_cmd,
          this,
          ros::TransportHints().tcpNoDelay()
  );
}

void drone_swarm_payload_t::decode(const char *buffer) {
  mini_odom_f32_t const * odom_ptr = (mini_odom_f32_t const *)buffer;
  pos_cmd_data_f32_t const * cmd_ptr = (pos_cmd_data_f32_t const *)(buffer+sizeof(mini_odom_f32_t));
  miniodom_to_odom<float>(*odom_ptr,latest_odom);
  latest_cmd = decode_pos_cmd(*cmd_ptr);
  latest_odom.header.stamp = ros::Time::now();
  latest_cmd.header.stamp = ros::Time::now();
  latest_odom.header.frame_id = "world";
  pub_odom.publish(latest_odom);
  pub_cmd.publish(latest_cmd);
}

void drone_swarm_payload_t::encode(char *buffer) {
  mini_odom_f32_t buffer_odom;
  odom_to_miniodom<float, int>(latest_odom,buffer_odom);
  pos_cmd_data_f32_t buffer_cmd = encode_pos_cmd<float>(latest_cmd);
  memcpy(buffer, (char*)&buffer_odom, sizeof(mini_odom_f32_t));
  memcpy(buffer+sizeof(mini_odom_f32_t), (char*)&buffer_cmd, sizeof(pos_cmd_data_f32_t));
}

int drone_swarm_payload_t::data_length() {
  return sizeof(mini_odom_f32_t) + sizeof(pos_cmd_data_f32_t);
}