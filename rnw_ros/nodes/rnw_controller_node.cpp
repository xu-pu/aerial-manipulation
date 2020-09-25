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

struct rnw_controller_t {

    rnw_config_t rnw_config;

    bool uav_odom_init = false;

    nav_msgs::Odometry latest_uav_odom;

    bool cone_state_init = false;

    rnw_msgs::ConeState latest_cone_state;

    AmTraj traj_generator;

    AmTraj rocking_generator;

    AmTraj zigzag_generator;

    rnw_planner_t rnw_planner;

    ros::Publisher pub_poly_traj;

    explicit rnw_controller_t(ros::NodeHandle & nh)
            : rnw_planner(nh,rnw_config),
              traj_generator(1024, 16, 0.4, 0.5, 0.5, 23, 0.02),
              zigzag_generator(1024, 16, 0.4, 1, 0.5, 23, 0.02),
              rocking_generator(1024, 16, 0.4, 0.5, 0.5, 23, 0.02)
    {
      rnw_config.load_from_ros(nh);
      pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/rnw/poly_traj",10,false);
      zigzag_generator = AmTraj(1024, 16, 0.4, rnw_config.zigzag.max_vel, rnw_config.zigzag.max_acc, 23, 0.02);
      traj_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.max_vel, rnw_config.rnw.max_acc, 23, 0.02);
      rocking_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.rocking_max_vel, rnw_config.rnw.rocking_max_acc, 23, 0.02);
    }

    void on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_uav_odom = *msg;
      uav_odom_init = true;
      rnw_planner.on_uav_odom(msg);
    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      latest_cone_state = *msg;
      cone_state_init = true;
      rnw_planner.on_cone_state(msg);
    }

    void on_trigger_insert( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[rnw] insert triggered!");

      if (!(uav_odom_init&&cone_state_init)) {
        ROS_ERROR_STREAM("[rnw] No odom, can't plan traj");
        return;
      }

      Vector3d pt_uav = pose2T(latest_uav_odom.pose.pose);
      Vector3d pt_tip = uav_utils::from_point_msg(latest_cone_state.tip);

      double z_planned_tcp = pt_tip.z() + rnw_config.rnw.hover_above_tip;
      double z_planned_uav = z_planned_tcp - rnw_config.X_tcp_cage.z(); // offset between imu and tcp

      Vector3d pt_tgt = pt_tip;
      pt_tgt.z() = z_planned_uav;

      Vector3d pt_inserted = pt_tgt;
      pt_inserted.z()  = pt_inserted.z() - rnw_config.rnw.hover_above_tip - rnw_config.insert_below_tip;

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC({pt_uav, pt_tgt, pt_inserted}, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,latest_uav_odom,ros::Time::now()));

    }

    void on_trigger_topple( std_msgs::HeaderConstPtr const & msg ) const {

      ROS_WARN_STREAM("[rnw] topple triggered!");

      if (!(uav_odom_init&&cone_state_init)) {
        ROS_ERROR_STREAM("[rnw] No odom, can't plan traj");
        return;
      }

      Matrix3d R_tip = odom2R(latest_cone_state.odom);
      Vector3d T_tip = uav_utils::from_point_msg(latest_cone_state.tip);
      Vector3d cur_pos = odom2T(latest_uav_odom);

      auto wpts_local = gen_wpts_insert_topple(rnw_config);

      // wpts_local is the desired positions of tcp in the tip frame
      // they need to be transformed to positions of the uav in the world frame
      auto waypoints = transform_pts(transform_pts(wpts_local,R_tip,T_tip),Matrix3d::Identity(),-rnw_config.X_tcp_cage);

      waypoints.insert(waypoints.begin(),cur_pos);

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,latest_uav_odom,ros::Time::now()));

    }

    void on_trigger_adjust_grip( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw] adjust_grip triggered!");
      rnw_planner.trigger_adjust_grip();
    }

    void on_trigger_adjust_nutation( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw] adjust_nutation triggered!");
      rnw_planner.trigger_adjust_nutation();
    }

    void on_trigger_adjust_yaw( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw] received adjust_yaw trigger!");
      rnw_planner.trigger_adjust_yaw();
    }

    void on_trigger_rnw( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[rnw] rnw triggered!");

      if (!(uav_odom_init&&cone_state_init)) {
        ROS_ERROR_STREAM("[rnw] No odom, can't plan traj");
        return;
      }

      rnw_planner.start_walking();

    }

    void on_trigger_zigzag( std_msgs::HeaderConstPtr const & msg ) const {

      ROS_WARN_STREAM("[rnw] zig-zag triggered!");

      if (!(uav_odom_init&&cone_state_init)) {
        ROS_ERROR_STREAM("[rnw] No odom, can't plan traj");
        return;
      }

      Matrix3d R = uav_utils::from_quaternion_msg(latest_uav_odom.pose.pose.orientation).toRotationMatrix();
      Vector3d T = uav_utils::from_point_msg(latest_uav_odom.pose.pose.position);

      vector<Vector3d> waypoints = gen_waypoint_zigzag(
              rnw_config.zigzag.cycles,
              rnw_config.zigzag.step_forward,
              rnw_config.zigzag.step_sideways
      );

      vector<Vector3d> wps = transform_pts(waypoints,R,T);

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = zigzag_generator.genOptimalTrajDTC(wps,v0,v0,v0,v0);
      pub_poly_traj.publish(to_ros_msg(traj,latest_uav_odom,ros::Time::now()));

    }

    void on_trigger_push_init( std_msgs::HeaderConstPtr const & msg ) const {

      ROS_WARN_STREAM("[rnw] push_init triggered!");

      if (!(uav_odom_init&&cone_state_init)) {
        ROS_ERROR_STREAM("[rnw] No odom, can't plan traj");
        return;
      }

      auto waypoints = gen_wpts_push_topple(latest_uav_odom,latest_cone_state,rnw_config);
      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);

      // align the object and uav
      double yaw_start = uav_yaw_from_odom(latest_uav_odom);
      double yaw_final = uav_yaw_from_cone_state(latest_cone_state);

      pub_poly_traj.publish(to_ros_msg(traj,yaw_start,yaw_final,ros::Time::now()));

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
      traj_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.max_vel, rnw_config.rnw.max_acc, 23, 0.02);
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

  rnw_controller_t rnw_controller(nh);

  auto timer = nh.createTimer(ros::Rate(30), &rnw_controller_t::rnw_planner_loop, &rnw_controller);

  ros::Subscriber sub_uav_odom = nh.subscribe<nav_msgs::Odometry>(
          "/odom/uav",
          10,
          &rnw_controller_t::on_uav_odom,
          &rnw_controller,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
          "/rnw/cone_state",
          10,
          &rnw_controller_t::on_cone_state,
          &rnw_controller,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_trigger_insert = nh.subscribe<std_msgs::Header>("/rnw/trigger/insert", 10, &rnw_controller_t::on_trigger_insert, &rnw_controller);
  ros::Subscriber sub_trigger_zigzag = nh.subscribe<std_msgs::Header>("/rnw/trigger/zigzag", 10, &rnw_controller_t::on_trigger_zigzag, &rnw_controller);
  ros::Subscriber sub_trigger_topple = nh.subscribe<std_msgs::Header>("/rnw/trigger/topple", 10, &rnw_controller_t::on_trigger_topple, &rnw_controller);
  ros::Subscriber sub_trigger_rnw = nh.subscribe<std_msgs::Header>("/rnw/trigger/rnw", 10, &rnw_controller_t::on_trigger_rnw, &rnw_controller);
  ros::Subscriber sub_trigger_push_init = nh.subscribe<std_msgs::Header>("/rnw/trigger/push_init", 10, &rnw_controller_t::on_trigger_push_init, &rnw_controller);
  ros::Subscriber sub_trigger_adjust_grip = nh.subscribe<std_msgs::Header>("/rnw/trigger/adjust_grip", 10, &rnw_controller_t::on_trigger_adjust_grip, &rnw_controller);
  ros::Subscriber sub_trigger_adjust_nutation = nh.subscribe<std_msgs::Header>("/rnw/trigger/adjust_nutation", 10, &rnw_controller_t::on_trigger_adjust_nutation, &rnw_controller);
  ros::Subscriber sub_trigger_adjust_yaw = nh.subscribe<std_msgs::Header>("/rnw/trigger/adjust_yaw", 10, &rnw_controller_t::on_trigger_adjust_yaw, &rnw_controller);

  dynamic_reconfigure::Server<rnw_ros::RNWConfig> server;
  server.setConfigDefault(rnw_controller.rnw_config.rnw.to_config());
  server.updateConfig(rnw_controller.rnw_config.rnw.to_config());
  server.setCallback([&]( rnw_ros::RNWConfig & config, uint32_t level ){
    rnw_controller.cfg_callback(config,level);
  });
  server.updateConfig(rnw_controller.rnw_config.rnw.to_config());

  ros::spin();

  ros::shutdown();

  return 0;

}