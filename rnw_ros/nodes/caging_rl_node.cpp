#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Joy.h>

#include "rnw_ros/cone_interface.h"
#include "rnw_ros/drone_interface.h"
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/rl_agent_interface.h"
#include "rnw_ros/RlConfig.h"

Vector3d tip_pos_to_drone_pos( Vector3d const & tip_pos, double yaw_rad, Vector3d const & flu_T_tcp ){
  Matrix3d R = uav_utils::rotz(yaw_rad);
  Vector3d offset = - R * flu_T_tcp;
  return tip_pos + offset;
}

struct caging_rl_t {

    rnw_config_t config;

    static constexpr int control_rate = 70;

    cone_interface_t cone;

    drone_interface_t drone;

    rl_agent_interface_t rl_agent;

    ros::Subscriber sub_gamepad_Y;

    ros::Subscriber sub_gamepad_X;

    ros::Subscriber sub_gamepad_RB;

    ros::Timer control_loop;

    ros::Time start_ts;

    caging_rl_t(): drone("drone1"), rl_agent(config) {

      ros::NodeHandle nh("~");

      config.load_from_ros(nh);

      sub_gamepad_Y = nh.subscribe<std_msgs::Header>("/gamepad/Y",10,&caging_rl_t::on_topple,this);

      sub_gamepad_X = nh.subscribe<std_msgs::Header>("/gamepad/X",10,&caging_rl_t::on_start,this);

      sub_gamepad_RB = nh.subscribe<std_msgs::Header>("/gamepad/RB",10,&caging_rl_t::on_stop,this);

      control_loop = nh.createTimer(ros::Rate(control_rate),&caging_rl_t::high_freq_ctrl_loop,this,false,false);

    }

    void on_topple( std_msgs::HeaderConstPtr const & msg ) const {

      ROS_WARN("[caging] topple triggered!");

      double yaw = cone_yaw(cone.latest_cone_state);

      vector<Vector3d> tip_wpts;

      double cur_nutation = rad2deg * cone.latest_cone_state.euler_angles.y;

      for ( double theta : range(cur_nutation,config.rnw.desired_nutation,5) ) {
        tip_wpts.emplace_back( cone.tip_at_nutation(deg2rad*theta) );
      }

      // build waypoints
      vector<Vector3d> drone_wpts;
      drone_wpts.emplace_back(drone.position());
      for ( auto const pt : tip_wpts ) {
        drone_wpts.emplace_back(tip_pos_to_drone_pos(pt,yaw,config.flu_T_tcp));
      }

      drone.follow_waypoints(drone_wpts,config.rl.yaw);

    }

    void on_start( std_msgs::HeaderConstPtr const & msg ){
      start_ts = ros::Time::now();
      control_loop.start();
    }

    void on_stop( std_msgs::HeaderConstPtr const & msg ){
      control_loop.stop();
    }

    void high_freq_ctrl_loop( ros::TimerEvent const & e ) const {

//      double dt = ( ros::Time::now() - start_ts ).toSec();
//      double vel = std::sin(dt);
//      Vector3d fake_cmd = action_to_cmd_vel(cone.latest_cone_state,{0,vel});

      if ( latency(rl_agent.latest_action) > 1 ) {
        ROS_WARN("[rl] action timeout");
        return;
      }

      drone.cmd_pos_vel_yaw(
              tip_pos_to_drone_pos(cone.tip(),drone.yaw(),config.flu_T_tcp),
              rl_agent.latest_cmd,
              config.rl.yaw
      );

    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"caging_rl_node");

  caging_rl_t node;

  dynamic_reconfigure::Server<rnw_ros::RlConfig> server;
  server.setConfigDefault(node.config.rl.to_config());
  server.updateConfig(node.config.rl.to_config());
  server.setCallback([&]( rnw_ros::RlConfig & config, uint32_t level ){
      ROS_WARN("RL setting updated");
      node.config.rl.set(config);
  });
  server.updateConfig(node.config.rl.to_config());

  ros::spin();

  ros::shutdown();

  return 0;

}