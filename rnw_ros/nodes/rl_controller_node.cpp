#include "ros/ros.h"
#include "rnw_ros/cone_interface.h"
#include "rnw_ros/rnw_utils.h"

#include <sensor_msgs/Joy.h>

Eigen::Vector3d action_to_cp_vel( rnw_msgs::ConeState const & cone_state, Eigen::Vector2d const & action ){
  auto R = calc_rnw_body_frame(cone_state);
  Eigen::Vector3d act( action.x(), action.y(), 0 );
  return R * act;
}

struct rl_rnw_controller_t {

    double action_freq = 30;

    double ctrl_freq = 30;

    cone_interface_t cone;

    ros::Publisher pub_rl_obs;

    ros::Publisher pub_cp_vel_cmd;

    ros::Subscriber sub_rl_action;

    ros::Timer timer_rl_loop;

    ros::Timer timer_ctrl_loop;

    sensor_msgs::Joy latest_action;

    sensor_msgs::Joy latest_obs;

    rl_rnw_controller_t(){

      ros::NodeHandle nh("~");

      pub_rl_obs = nh.advertise<sensor_msgs::Joy>("/rl_agent/observation",100);

      pub_cp_vel_cmd = nh.advertise<geometry_msgs::Vector3Stamped>("/rl_agent/action_cp_vel",100);

      sub_rl_action = nh.subscribe<sensor_msgs::Joy>(
              "/rl_agent/action",
              1,
              &rl_rnw_controller_t::on_rl_agent_action,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      timer_rl_loop = nh.createTimer(ros::Rate(action_freq), &rl_rnw_controller_t::rl_loop, this );

      timer_ctrl_loop = nh.createTimer( ros::Rate(ctrl_freq), &rl_rnw_controller_t::ctrl_loop, this );

    }

    void on_rl_agent_action( sensor_msgs::JoyConstPtr const & msg ){
      latest_action = *msg;
      if ( msg->axes.size() != 2 ) {
        ROS_ERROR("[rl] incorrect action size, %lu", msg->axes.size());
        return;
      }

      Eigen::Vector2d action { msg->axes.at(0), msg->axes.at(1) };

      Eigen::Vector3d cp_vel = action_to_cp_vel(cone.latest_cone_state,action);

      geometry_msgs::Vector3Stamped rst;
      rst.header = msg->header;
      rst.vector = uav_utils::to_vector3_msg(cp_vel);
      pub_cp_vel_cmd.publish(rst);

    }

    void rl_loop( ros::TimerEvent const & e ){

      if ( !cone.odom_in_time() ) {
        ROS_WARN("[rl] no cone state!");
        return;
      }

      auto const & c = cone.latest_cone_state;

      latest_obs.header = cone.latest_cone_state.header;
      latest_obs.axes.resize(6+5);

      latest_obs.axes.at(0) = (float)c.euler_angles.x;
      latest_obs.axes.at(1) = (float)c.euler_angles.y;
      latest_obs.axes.at(2) = (float)c.euler_angles.z;

      latest_obs.axes.at(4) = (float)c.euler_angles_velocity.x;
      latest_obs.axes.at(5) = (float)c.euler_angles_velocity.y;
      latest_obs.axes.at(6) = (float)c.euler_angles_velocity.z;

      latest_obs.axes.at(7) = (float)c.radius;
      latest_obs.axes.at(8) = (float)c.radius;
      latest_obs.axes.at(9) = 0;
      latest_obs.axes.at(10) = -(float)c.radius;
      latest_obs.axes.at(11) = 1.5;

      pub_rl_obs.publish(latest_obs);

    }

    void ctrl_loop( ros::TimerEvent const & e ){
      // send to quads
    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"rl_controller_node");

  ros::NodeHandle nh("~");

  rl_rnw_controller_t controller;

  ros::spin();

  ros::shutdown();

  return 0;

}