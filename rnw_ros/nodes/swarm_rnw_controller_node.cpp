#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <am_traj/am_traj.hpp>
#include <am_traj/ros_msgs.h>
#include <uav_utils/converters.h>

#include "rnw_ros/RNWConfig.h"
#include "rnw_ros/traj_uitls.h"
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_planner_v2.h"
#include "rnw_ros/swarm_interface.h"

struct swarm_planner_t {

    rnw_config_t & rnw_config;

    swarm_interface_t swarm_interface;

    AmTraj rocking_generator;

    swarm_planner_t(ros::NodeHandle & nh, rnw_config_t & cfg ) :
            rnw_config(cfg),
            swarm_interface(nh),
            rocking_generator(1024, 16, 0.4, 0.5, 0.5, 23, 0.02)
    {
      rocking_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.rocking_max_vel, rnw_config.rnw.rocking_max_acc, 23, 0.02);
    }

    /**
     * Execute the command, return an estimated time cost
     * @param cmd
     * @return estimated duration (sec)
     */
    double execute_cmd( rnw_command_t const & cmd ){
      return 1;
    }

};

struct rnw_node_t {

    rnw_config_t & rnw_config;

    swarm_planner_t swarm_planner;

    rnw_planner_v2_t rnw_planner;

    ros::Timer timer;

    ros::Subscriber sub_cone_state;

    ros::Subscriber sub_trigger_start;

    ros::Subscriber sub_trigger_abort;

    ros::Time cmd_start_time;

    ros::Duration cmd_duration;

    rnw_node_t(ros::NodeHandle & nh, rnw_config_t & cfg )
            : rnw_config(cfg),
              rnw_planner(cfg),
              swarm_planner(nh,cfg)
    {

      timer = nh.createTimer(ros::Rate(30), &rnw_node_t::rnw_planner_loop, this);

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              10,
              &rnw_node_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_trigger_start = nh.subscribe<std_msgs::Header>(
              "/rnw/start",
              10,
              &rnw_node_t::on_start,
              this
      );

      sub_trigger_abort = nh.subscribe<std_msgs::Header>(
              "/abort",
              10,
              &rnw_node_t::on_abort,
              this
      );

    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      rnw_planner.on_cone_state(msg);
    }

    void on_start( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw_planner] start triggered!");
      rnw_planner.start_walking();
    }

    void on_abort( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw_planner] abort triggered!");
      rnw_planner.stop_walking();
    }

    /**
     * This loop checks commands (RockingCmd) from rnw_planner
     * If there is pending RockingCmd, execute
     * After execution, rnw_planner.cmd_complete()
     * @param event
     */
    void rnw_planner_loop( const ros::TimerEvent &event ){
      rnw_planner.spin();
      switch(rnw_planner.cmd_fsm){
        case rnw_planner_v2_t::cmd_fsm_e::executing:
        {
          if (ros::Time::now() > cmd_start_time + cmd_duration) {
            rnw_planner.cmd_complete();
          }
        }
          break;
        case rnw_planner_v2_t::cmd_fsm_e::pending:
        {
          rnw_command_t cmd = rnw_planner.take_cmd();
          double time_to_go = swarm_planner.execute_cmd(cmd);
          cmd_start_time = ros::Time::now();
          cmd_duration = ros::Duration(time_to_go);
        }
          break;
        default: break;
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"swarm_rnw_controller_node");

  ros::NodeHandle nh("~");

  rnw_config_t rnw_config;
  rnw_config.load_from_ros(nh);

  rnw_node_t rnw_node(nh, rnw_config);

  dynamic_reconfigure::Server<rnw_ros::RNWConfig> server;
  server.setConfigDefault(rnw_config.rnw.to_config());
  server.updateConfig(rnw_config.rnw.to_config());
  server.setCallback([&]( rnw_ros::RNWConfig & config, uint32_t level ){
      ROS_WARN_STREAM("[rnw] re-config rnw");
      rnw_config.rnw.hover_above_tip = config.hover_above_tip;
      rnw_config.rnw.insertion_depth = config.insertion_depth;
      rnw_config.rnw.topple_init = config.topple_init;
      rnw_config.rnw.desired_nutation = config.desired_nutation;
      rnw_config.rnw.desired_grip_depth = config.desired_grip_depth;
      rnw_config.rnw.tau = config.tau;
      rnw_config.rnw.max_vel = config.max_vel;
      rnw_config.rnw.max_acc = config.max_acc;
      rnw_config.rnw.rocking_max_vel = config.rocking_max_vel;
      rnw_config.rnw.rocking_max_acc = config.rocking_max_acc;
      rnw_node.swarm_planner.rocking_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.rocking_max_vel, rnw_config.rnw.rocking_max_acc, 23, 0.02);
      ROS_INFO_STREAM(rnw_config.rnw.to_string());
  });
  server.updateConfig(rnw_config.rnw.to_config());

  ros::spin();

  ros::shutdown();

  return 0;

}