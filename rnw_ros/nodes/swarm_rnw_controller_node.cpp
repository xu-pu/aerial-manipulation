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
#include "rnw_ros/rnw_planner.h"

struct rnw_interface_t {

    rnw_config_t rnw_config;

    bool uav_odom_init = false;

    nav_msgs::Odometry latest_uav_odom;

    bool cone_state_init = false;

    AmTraj rocking_generator;

    rnw_planner_t rnw_planner;

    ros::Publisher pub_poly_traj;

    ros::Timer timer;

    ros::Subscriber sub_uav_odom;

    ros::Subscriber sub_cone_state;

    ros::Subscriber sub_trigger_rnw;

    explicit rnw_interface_t(ros::NodeHandle & nh)
            : rnw_planner(nh,rnw_config),
              rocking_generator(1024, 16, 0.4, 0.5, 0.5, 23, 0.02)
    {

      rnw_config.load_from_ros(nh);

      rocking_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.rocking_max_vel, rnw_config.rnw.rocking_max_acc, 23, 0.02);

      timer = nh.createTimer(ros::Rate(30), &rnw_interface_t::rnw_planner_loop, this);

      pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/rnw/poly_traj",10,false);

      sub_uav_odom = nh.subscribe<nav_msgs::Odometry>(
              "/odom/uav",
              10,
              &rnw_interface_t::on_uav_odom,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              10,
              &rnw_interface_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_trigger_rnw = nh.subscribe<std_msgs::Header>(
              "/rnw/trigger/rnw",
              10,
              &rnw_interface_t::on_trigger_rnw,
              this
      );

    }

    void on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_uav_odom = *msg;
      uav_odom_init = true;
      rnw_planner.on_uav_odom(msg);
    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      cone_state_init = true;
      rnw_planner.on_cone_state(msg);
    }

    void on_trigger_rnw( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[rnw] rnw triggered!");

      if (!(uav_odom_init&&cone_state_init)) {
        ROS_ERROR_STREAM("[rnw] No odom, can't plan traj");
        return;
      }

      rnw_planner.start_walking();

    }

    void cfg_callback( rnw_ros::RNWConfig & config, uint32_t level ){
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
      rocking_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.rocking_max_vel, rnw_config.rnw.rocking_max_acc, 23, 0.02);
      ROS_INFO_STREAM(rnw_config.rnw.to_string());
    }

    ///////////////////////////////////////////////////
    //// Here we handle the rnw_planner
    ///////////////////////////////////////////////////

    ros::Time cmd_start_time;

    ros::Duration cmd_duration;

    /**
     * This loop checks commands (RockingCmd) from rnw_planner
     * If there is pending RockingCmd, execute
     * After execution, rnw_planner.cmd_complete()
     * @param event
     */
    void rnw_planner_loop( const ros::TimerEvent &event ){
      rnw_planner.spin();
      if ( rnw_planner.rnw_cmd.fsm == rnw_cmd_t::fsm_executing ) {
        if (ros::Time::now() > cmd_start_time + cmd_duration ) {
          rnw_planner.cmd_complete();
        }
      }
      else if ( rnw_planner.rnw_cmd.fsm == rnw_cmd_t::fsm_pending ) {

        rnw_cmd_t * cmd = rnw_planner.take_cmd();

        vector<Vector3d> waypoints = {};
        waypoints.push_back(pose2T(latest_uav_odom.pose.pose));
        waypoints.push_back(cmd->setpoint_uav);

        if (check_waypoints(waypoints)) {
          Vector3d v0 = Vector3d::Zero();
          Trajectory traj = rocking_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
          double yaw_start = uav_yaw_from_odom(latest_uav_odom);
          double yaw_final = cmd->desired_yaw;
          pub_poly_traj.publish(to_ros_msg(traj,yaw_start,yaw_final,ros::Time::now()));
          cmd_start_time = ros::Time::now();
          cmd_duration = ros::Duration(traj.getTotalDuration());
        }
        else {
          ROS_ERROR_STREAM("[rnw] trajectory is too short! Did not execute!");
          rnw_planner.cmd_complete();
        }

      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_controller_node");

  ros::NodeHandle nh("~");

  rnw_interface_t rnw_interface(nh);

  dynamic_reconfigure::Server<rnw_ros::RNWConfig> server;
  server.setConfigDefault(rnw_interface.rnw_config.rnw.to_config());
  server.updateConfig(rnw_interface.rnw_config.rnw.to_config());
  server.setCallback([&]( rnw_ros::RNWConfig & config, uint32_t level ){
    rnw_interface.cfg_callback(config, level);
  });
  server.updateConfig(rnw_interface.rnw_config.rnw.to_config());

  ros::spin();

  ros::shutdown();

  return 0;

}