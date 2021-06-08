#include "uwb_transceiver/drone_swarm_payload.h"
#include <djiros/DroneArmControl.h>

using namespace uwb_comm;

std_msgs::Header gen_trigger_msg(){
  std_msgs::Header msg;
  msg.frame_id = "world";
  msg.stamp = ros::Time::now();
  return msg;
}

enum trigger_e {
    trigger_idle = 0,
    trigger_arm_motors = 1,
};

struct n3ctrl_state_t {
    uint8_t state;
    uint32_t last_traj_id;
};

struct forward_data_t {

    ////////////// odom /////////////

    uint32_t odom_seq;

    float odom_p_x;
    float odom_p_y;
    float odom_p_z;

    float odom_v_x;
    float odom_v_y;
    float odom_v_z;

    float odom_q_x;
    float odom_q_y;
    float odom_q_z;
    float odom_q_w;

    float odom_w_x;
    float odom_w_y;
    float odom_w_z;

    ////////////// cmd /////////////

    uint32_t cmd_seq;

    float cmd_p_x;
    float cmd_p_y;
    float cmd_p_z;

    float cmd_v_x;
    float cmd_v_y;
    float cmd_v_z;

    float cmd_a_x;
    float cmd_a_y;
    float cmd_a_z;

    float cmd_yaw;
    float cmd_yaw_dot;

    ////////////// trigger /////////////

    uint8_t trigger;

};

nav_msgs::Odometry extract_odom( forward_data_t const & data ){

  nav_msgs::Odometry odom;

  odom.header.seq = data.odom_seq;
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();

  odom.pose.pose.position.x = data.odom_p_x;
  odom.pose.pose.position.y = data.odom_p_y;
  odom.pose.pose.position.z = data.odom_p_z;

  odom.twist.twist.linear.x = data.odom_v_x;
  odom.twist.twist.linear.y = data.odom_v_y;
  odom.twist.twist.linear.z = data.odom_v_z;

  odom.pose.pose.orientation.x = data.odom_q_x;
  odom.pose.pose.orientation.y = data.odom_q_y;
  odom.pose.pose.orientation.z = data.odom_q_z;
  odom.pose.pose.orientation.w = data.odom_q_w;

  odom.twist.twist.angular.x = data.odom_w_x;
  odom.twist.twist.angular.y = data.odom_w_y;
  odom.twist.twist.angular.z = data.odom_w_z;

  return odom;

}

quadrotor_msgs::PositionCommand extract_cmd( forward_data_t const & dat ){

  quadrotor_msgs::PositionCommand msg;

  msg.header.seq = dat.cmd_seq;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time::now();

  msg.position.x = dat.cmd_p_x;
  msg.position.y = dat.cmd_p_y;
  msg.position.z = dat.cmd_p_z;

  msg.velocity.x = dat.cmd_v_x;
  msg.velocity.y = dat.cmd_v_y;
  msg.velocity.z = dat.cmd_v_z;

  msg.acceleration.x = dat.cmd_a_x;
  msg.acceleration.y = dat.cmd_a_y;
  msg.acceleration.z = dat.cmd_a_z;

  msg.yaw = dat.cmd_yaw;
  msg.yaw_dot = dat.cmd_yaw_dot;

  return msg;

}

void encode_odom( forward_data_t & data, nav_msgs::Odometry const & odom ){

  data.odom_seq = odom.header.seq;

  data.odom_p_x = odom.pose.pose.position.x;
  data.odom_p_y = odom.pose.pose.position.y;
  data.odom_p_z = odom.pose.pose.position.z;

  data.odom_v_x = odom.twist.twist.linear.x;
  data.odom_v_y = odom.twist.twist.linear.y;
  data.odom_v_z = odom.twist.twist.linear.z;

  data.odom_q_x = odom.pose.pose.orientation.x;
  data.odom_q_y = odom.pose.pose.orientation.y;
  data.odom_q_z = odom.pose.pose.orientation.z;
  data.odom_q_w = odom.pose.pose.orientation.w;

  data.odom_w_x = odom.twist.twist.angular.x;
  data.odom_w_y = odom.twist.twist.angular.y;
  data.odom_w_z = odom.twist.twist.angular.z;

}

void encode_cmd( forward_data_t & dat, quadrotor_msgs::PositionCommand const & msg ){

  dat.cmd_seq = msg.header.seq;

  dat.cmd_p_x = msg.position.x;
  dat.cmd_p_y = msg.position.y;
  dat.cmd_p_z = msg.position.z;

  dat.cmd_v_x = msg.velocity.x;
  dat.cmd_v_y = msg.velocity.y;
  dat.cmd_v_z = msg.velocity.z;

  dat.cmd_a_x = msg.acceleration.x;
  dat.cmd_a_y = msg.acceleration.y;
  dat.cmd_a_z = msg.acceleration.z;

  dat.cmd_yaw = msg.yaw;
  dat.cmd_yaw_dot = msg.yaw_dot;

}

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

void drone_swarm_payload_t::on_triggers( const std_msgs::UInt8ConstPtr & msg ) {
  if ( msg->data > 0 ) {
    while ( !trigger_queue.empty() ) { trigger_queue.pop(); }
    ROS_WARN("[uwb_master] trigger %u received", msg->data);
    while ( trigger_queue.size() < 5 ) { trigger_queue.push(msg->data); }
  }
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

  sub_triggers = nh.subscribe<std_msgs::UInt8>(
          "trigger",
          100,
          &drone_swarm_payload_t::on_triggers,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  pub_n3ctrl = nh.advertise<n3ctrl::N3CtrlState>("n3ctrl",100);

}

void drone_swarm_payload_t::slave_decode(const char *buffer) {

  forward_data_t const & data = *((forward_data_t const *)buffer);

  ///////// process odom
  if ( data.odom_seq != latest_odom.header.seq ) {
    latest_odom = extract_odom(data);
    latest_odom.header.stamp = ros::Time::now() - odom_latency;
    pub_odom.publish(latest_odom);
  }

  //////// process cmd
  // refresh cmd (or not)
  if ( data.cmd_seq > 0 && data.cmd_seq != latest_cmd.header.seq ) {
    latest_cmd = extract_cmd(data);
  }
  // send out command (or not)
  if ( (ros::Time::now() - latest_cmd.header.stamp).toSec() < msg_timeout_sec ) {
    pub_cmd.publish(latest_cmd);
  }

  //////// triggers
  switch (data.trigger) {
    case trigger_arm_motors:
      arm_motors();
      break;
    default:
      break;
  }

}

bool drone_swarm_payload_t::master_encode(char *buffer) {
  forward_data_t data {};
  encode_odom(data,latest_odom);
  encode_cmd(data,latest_cmd);
  data.trigger = 0;
  if ( !trigger_queue.empty() ) {
    data.trigger = trigger_queue.back();
    trigger_queue.pop();
  }
  memcpy(buffer, (char*)(&data), sizeof(forward_data_t));
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
  return sizeof(forward_data_t);
}

int drone_swarm_payload_t::data_length_slave() {
  return sizeof(n3ctrl_state_t);
}

void drone_swarm_payload_t::arm_motors(){
  ROS_WARN("[uwb_receiver] arm motors!!!");
  djiros::DroneArmControl arm;
  arm.request.arm = 1;
  ros::service::call("/djiros/drone_arm_control",arm);
}