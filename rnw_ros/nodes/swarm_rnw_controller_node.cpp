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

    quadrotor_msgs::PolynomialTrajectory genetate_uav_traj( nav_msgs::Odometry const & from, Vector3d const & to ) const {

      constexpr double epsi = 0.05;

      Vector3d pt_start = uav_utils::from_point_msg(from.pose.pose.position);

      double dist = (to-pt_start).norm();

      if ( dist < epsi ) {
        return gen_setpoint_traj(from,to,0.5);
      }
      else {
        // plan traj
        vector<Vector3d> wpts;
        wpts.emplace_back(pt_start);
        wpts.emplace_back((pt_start+to)/2);
        wpts.emplace_back(to);
        Vector3d v0 = Vector3d::Zero();
        auto traj = rocking_generator.genOptimalTrajDTC(wpts,v0,v0,v0,v0);
        return to_ros_msg(traj,from,ros::Time::now());
      }

    }

    /**
     * Execute the command, return an estimated time cost
     * @param cmd
     * @return estimated duration (sec)
     */
    double execute_cmd( rnw_command_t const & cmd ) const {

      if ( !swarm_interface.initialized() ) {
        ROS_ERROR_STREAM("swarm odom not properly initialized");
        return 0;
      }

      double rad = deg2rad * 0.5 * rnw_config.swarm.angle;
      Vector3d setpoint1 = calc_pt_at_cp_frame(cmd.control_point_setpoint,cmd.heading,rnw_config.swarm.cable1,rad);
      Vector3d setpoint2 = calc_pt_at_cp_frame(cmd.control_point_setpoint,cmd.heading,rnw_config.swarm.cable2,-rad);
      nav_msgs::Odometry odom1 = swarm_interface.latest_odom_drone1;
      nav_msgs::Odometry odom2 = swarm_interface.latest_odom_drone2;

      quadrotor_msgs::PolynomialTrajectory traj1 = genetate_uav_traj(odom1,setpoint1);
      quadrotor_msgs::PolynomialTrajectory traj2 = genetate_uav_traj(odom2,setpoint2);

      swarm_interface.send_traj(traj1,traj2);

      double dt1 = get_traj_duration(traj1);
      double dt2 = get_traj_duration(traj2);

      ROS_INFO_STREAM("[rnw_planner] execute cmd #" << cmd.cmd_idx << ", drone1 " << dt1 << "s, drone2 " << dt2 << "s");

      return std::max(dt1,dt2);

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

      timer = nh.createTimer(ros::Rate(30), &rnw_node_t::spin, this);

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
     * Spin the rnw_planner.
     * If there is pending command, execute.
     * After execution, rnw_planner.cmd_complete()
     */
    void spin(const ros::TimerEvent &event ){
      rnw_planner.spin();
      if ( rnw_planner.cmd_fsm == rnw_planner_v2_t::cmd_fsm_e::executing ) {
        if (ros::Time::now() > cmd_start_time + cmd_duration) {
          rnw_planner.cmd_complete();
        }
      }
      else if ( rnw_planner.cmd_fsm == rnw_planner_v2_t::cmd_fsm_e::pending ) {
        rnw_command_t cmd = rnw_planner.take_cmd();
        double time_to_go = swarm_planner.execute_cmd(cmd);
        cmd_start_time = ros::Time::now();
        cmd_duration = ros::Duration(time_to_go);
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