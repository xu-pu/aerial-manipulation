#include "rnw_ros/rl_agent_interface.h"

#include "rnw_ros/rnw_utils.h"

rl_agent_interface_t::rl_agent_interface_t( rnw_config_t & cfg ) : config(cfg) {

  ros::NodeHandle nh("~");

  pub_obs = nh.advertise<sensor_msgs::Joy>("/rl_agent/observation", 10);

  pub_cmd_vel = nh.advertise<geometry_msgs::Vector3Stamped>("/rl_agent/action_cp_vel",100);

  sub_action = nh.subscribe<sensor_msgs::Joy>(
          "/rl_agent/action",
          1,
          &rl_agent_interface_t::on_action,
          this,
          ros::TransportHints().tcpNoDelay()
  );

  timer_obs = nh.createTimer( ros::Rate(obs_rate), &rl_agent_interface_t::on_observation, this );

}

void rl_agent_interface_t::on_action( const sensor_msgs::JoyConstPtr & msg ) {

  latest_action = *msg;
  if ( msg->axes.size() != 2 ) {
    ROS_ERROR("[rl] incorrect action size, %lu", msg->axes.size());
    return;
  }

  latest_action = *msg;

  double x = config.rl.enable_x ? msg->axes.at(0) : 0;
  double y = config.rl.enable_y ? msg->axes.at(1) : 0;

  Eigen::Vector2d action { x, y };
  Eigen::Vector3d cmd_vel = action_to_cmd_vel(cone.latest_cone_state, action*config.rl.action_scale);
  latest_cmd = cmd_vel;

  geometry_msgs::Vector3Stamped rst;
  rst.header = msg->header;
  rst.vector = uav_utils::to_vector3_msg(cmd_vel);
  pub_cmd_vel.publish(rst);

}

void rl_agent_interface_t::on_observation( const ros::TimerEvent & e ) {

  if ( !cone.odom_in_time() ) {
    ROS_WARN("[rl] no cone state!");
    return;
  }

  auto const & c = cone.latest_cone_state;

  latest_obs.header = cone.latest_cone_state.header;
  //latest_obs.axes.resize(6+5);
  latest_obs.axes.resize(6);

  latest_obs.axes.at(0) = (float)(c.euler_angles.x-M_PI_2);
  latest_obs.axes.at(1) = (float)c.euler_angles.y;
  latest_obs.axes.at(2) = (float)c.euler_angles.z;

  latest_obs.axes.at(3) = (float)c.euler_angles_velocity.x;
  latest_obs.axes.at(4) = (float)c.euler_angles_velocity.y;
  latest_obs.axes.at(5) = (float)c.euler_angles_velocity.z;

//  latest_obs.axes.at(6) = (float)c.radius;
//  latest_obs.axes.at(7) = (float)c.radius;
//  latest_obs.axes.at(8) = 0;
//  latest_obs.axes.at(9) = -(float)c.radius;
//  latest_obs.axes.at(10) = (float)c.height;

  pub_obs.publish(latest_obs);

}