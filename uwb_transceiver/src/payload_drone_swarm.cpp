#include "uwb_transceiver/drone_swarm_payload.h"
#include "uwb_transceiver/mini_odom.h"

using namespace uwb_comm;

struct n3ctrl_state_t {
    uint8_t state;
    uint32_t last_traj_id;
};

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
using mini_odom_f32_t = Mini_odom<float,uint32_t>;

drone_swarm_payload_t::drone_swarm_payload_t(ros::NodeHandle & _nh) : nh(_nh) {
  latest_cmd.header.seq = 0;
}

bool drone_swarm_payload_t::odom_on_time() const {
  return ( ros::Time::now() - latest_odom.header.stamp ).toSec() < msg_timeout_sec;
}

bool drone_swarm_payload_t::n3ctrl_on_time() const {
  return ( ros::Time::now() - latest_n3ctrl.header.stamp ).toSec() < msg_timeout_sec;
}

void drone_swarm_payload_t::on_cmd(const quadrotor_msgs::PositionCommandConstPtr & msg) {
  latest_cmd = *msg;
}

void drone_swarm_payload_t::on_odom(const nav_msgs::OdometryConstPtr & msg) {
  latest_odom = *msg;
}

void drone_swarm_payload_t::on_n3ctrl(const n3ctrl::N3CtrlStateConstPtr & msg) {
  latest_n3ctrl = *msg;
}

void drone_swarm_payload_t::init_as_slave() {

  pub_odom = nh.advertise<nav_msgs::Odometry>("odom",100);

  pub_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd",100);

  sub_n3ctrl = nh.subscribe<n3ctrl::N3CtrlState>(
          "n3ctrl",
          100,
          &drone_swarm_payload_t::on_n3ctrl,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  nh.getParam("odom_latency_ms",odom_latency_ms);
  odom_latency = ros::Duration(odom_latency_ms/1000.);

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

  pub_n3ctrl = nh.advertise<n3ctrl::N3CtrlState>("n3ctrl",100);

}

void drone_swarm_payload_t::slave_decode(const char *buffer) {

  // process odom
  auto * odom_ptr = (mini_odom_f32_t const *)buffer;
  if ( odom_ptr->seq != latest_odom.header.seq ) {
    miniodom_to_odom<float>(*odom_ptr,latest_odom);
    latest_odom.header.stamp = ros::Time::now() - odom_latency;
    latest_odom.header.frame_id = "world";
    pub_odom.publish(latest_odom);
  }
  else {
    //ROS_INFO("[uwb] odom #%u again", odom_ptr->seq);
  }

  // process cmd
  auto * cmd_ptr = (pos_cmd_data_f32_t const *)(buffer+sizeof(mini_odom_f32_t));
  if ( cmd_ptr->seq == 0 ) {
    // seq == 0 is ignored, might be uninitialized empty command
    return;
  }
  else if ( cmd_ptr->seq != latest_cmd.header.seq ) {
    latest_cmd = decode_pos_cmd(*cmd_ptr);
    latest_cmd.header.stamp = ros::Time::now();
  }
  else if ( (ros::Time::now() - latest_cmd.header.stamp).toSec() > msg_timeout_sec ) {
    // did not receive new cmd for one full second, stop sending the same cmd
    //ROS_WARN_STREAM("[UWB][slave] no new command");
    return;
  }
  //ROS_INFO_STREAM("[UWB][slave] send out cmd with seq " << latest_cmd.header.seq);
  pub_cmd.publish(latest_cmd);

}

bool drone_swarm_payload_t::master_encode(char *buffer) {
  mini_odom_f32_t buffer_odom;
  odom_to_miniodom<float,uint32_t>(latest_odom,buffer_odom);
  pos_cmd_data_f32_t buffer_cmd = encode_pos_cmd<float>(latest_cmd);
  memcpy(buffer, (char*)&buffer_odom, sizeof(mini_odom_f32_t));
  memcpy(buffer+sizeof(mini_odom_f32_t), (char*)&buffer_cmd, sizeof(pos_cmd_data_f32_t));
  return odom_on_time();
}

bool drone_swarm_payload_t::slave_encode(char *buffer) {
  n3ctrl_state_t dat {};
  dat.state = latest_n3ctrl.state;
  dat.last_traj_id = latest_n3ctrl.last_traj_id;
  memcpy(buffer, (char*)&dat, sizeof(n3ctrl_state_t));
  return n3ctrl_on_time();
}

void drone_swarm_payload_t::master_decode(const char *buffer) {
  auto const * ptr = (n3ctrl_state_t const *)buffer;
  n3ctrl_state_t dat = *ptr;
  latest_n3ctrl.header.stamp = ros::Time::now();
  latest_n3ctrl.header.frame_id = "world";
  latest_n3ctrl.state = dat.state;
  latest_n3ctrl.last_traj_id = dat.last_traj_id;
  pub_n3ctrl.publish(latest_n3ctrl);
}

int drone_swarm_payload_t::data_length_master() {
  return sizeof(mini_odom_f32_t) + sizeof(pos_cmd_data_f32_t);
}

int drone_swarm_payload_t::data_length_slave() {
  return sizeof(n3ctrl_state_t);
}