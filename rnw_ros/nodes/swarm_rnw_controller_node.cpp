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
#include "rnw_ros/cone_interface.h"

struct ref_frame_t {

    Matrix3d R;

    Vector3d T;

    ref_frame_t( Matrix3d const & R_, Vector3d const & T_ ): R(R_), T(T_) { }

    ref_frame_t(){
      R.setIdentity();
      T.setZero();
    }

    Vector3d operator*( Vector3d const & pt ) const {
      return R * pt + T;
    }

};

ref_frame_t calc_control_frame( Vector3d const & tip, double heading ){
  return {
          Eigen::AngleAxisd(heading,Vector3d::UnitZ()).toRotationMatrix(),
          tip
  };
}

struct rnw_node_t {

    rnw_config_t & rnw_config;

    swarm_interface_t swarm;

    cone_interface_t cone;

    rnw_planner_v2_t rnw_planner;

    ros::Timer timer;

    ros::Subscriber sub_cone_state;

    ros::Subscriber sub_trigger_start;

    ros::Subscriber sub_trigger_abort;

    ros::Subscriber sub_trigger_swing_up;

    ros::Subscriber sub_trigger_land;

    ros::Subscriber sub_trigger_arm;

    rnw_node_t(ros::NodeHandle & nh, rnw_config_t & cfg )
            : rnw_config(cfg),
              rnw_planner(cfg),
              swarm(nh)
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
              "/gamepad/X",
              10,
              &rnw_node_t::on_start,
              this
      );

      sub_trigger_swing_up = nh.subscribe<std_msgs::Header>(
              "/gamepad/Y",
              10,
              &rnw_node_t::on_swing_up,
              this
      );

      sub_trigger_land = nh.subscribe<std_msgs::Header>(
              "/gamepad/A",
              10,
              &rnw_node_t::on_land,
              this
      );

      sub_trigger_abort = nh.subscribe<std_msgs::Header>(
              "/abort",
              10,
              &rnw_node_t::on_abort,
              this
      );

      sub_trigger_arm = nh.subscribe<std_msgs::Header>(
              "/gamepad/LB",
              10,
              &rnw_node_t::on_arm_motors,
              this
      );

    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      rnw_planner.on_cone_state(msg);
    }

    void on_start( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw_planner] start triggered!");
      if (swarm.ready()) {
        rnw_planner.start_walking();
      }
      else {
        ROS_WARN_STREAM("[rnw_planner] swarm not ready, can not start rnw!");
      }
    }

    void on_abort( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw_planner] abort triggered!");
      rnw_planner.stop_walking();
    }

    void on_land( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw_planner] land triggered!");
      rnw_planner.stop_walking();

      double heading = cone.latest_cone_state.euler_angles.x + M_PI_2;
      double spread = deg2rad * 0.5 * rnw_config.swarm.angle;

      Vector3d rest_tip = cone.tip_at_nutation(M_PI_2);
      ref_frame_t rest_frame = calc_control_frame(rest_tip,heading);

      Vector3d suspend_pt_drone1 = swarm.drone1.cable_length * Vector3d(0, sin(-spread), cos(-spread));
      Vector3d suspend_pt_drone2 = swarm.drone2.cable_length * Vector3d(0, sin(spread), cos(spread));
      Vector3d rest_pt_drone1 = swarm.drone1.cable_length * 0.8 * Vector3d(0, sin(-M_PI_2), cos(-M_PI_2));
      Vector3d rest_pt_drone2 = swarm.drone2.cable_length * 0.8 * Vector3d(0, sin(M_PI_2), cos(M_PI_2));

      vector<Vector3d> waypoints1;
      vector<Vector3d> waypoints2;
      if ( cone.latest_cone_state.euler_angles.y > deg2rad * 80 ) {
        // put down the object
        waypoints1.emplace_back(rest_frame*suspend_pt_drone1);
        waypoints2.emplace_back(rest_frame*suspend_pt_drone2);
      }

      // landing spot
      waypoints1.emplace_back(rest_frame*rest_pt_drone1);
      waypoints2.emplace_back(rest_frame*rest_pt_drone2);

      // disarm
      waypoints1.emplace_back(rest_frame*rest_pt_drone1);
      waypoints2.emplace_back(rest_frame*rest_pt_drone2);
      waypoints1.back().z() = -0.5;
      waypoints2.back().z() = -0.5;

      auto setting = drone_interface_t::create_setting(0.5,0.5);
      swarm.drone1.execute_trajectory(swarm.drone1.plan(setting,waypoints1),true);
      swarm.drone2.execute_trajectory(swarm.drone2.plan(setting,waypoints2),true);

    }

    void on_swing_up(std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[rnw_planner] swing up triggered!");
      rnw_planner.stop_walking();

      double heading = cone.latest_cone_state.euler_angles.x + M_PI_2;
      double cur_nutation = rad2deg * cone.latest_cone_state.euler_angles.y;
      double spread = deg2rad * 0.5 * rnw_config.swarm.angle;

      Vector3d suspend_pt_drone1 = swarm.drone1.cable_length * Vector3d(0, sin(-spread), cos(-spread));
      Vector3d suspend_pt_drone2 = swarm.drone2.cable_length * Vector3d(0, sin(spread), cos(spread));

      vector<Vector3d> waypoints_drone1;
      vector<Vector3d> waypoints_drone2;
      for ( double theta : range(cur_nutation,rnw_config.rnw.desired_nutation,10) ) {
        Vector3d tip = cone.tip_at_nutation(theta * deg2rad);
        ref_frame_t control_frame = calc_control_frame(tip,heading);
        waypoints_drone1.emplace_back(control_frame * suspend_pt_drone1);
        waypoints_drone2.emplace_back(control_frame * suspend_pt_drone2);
      }

      auto setting = drone_interface_t::create_setting(1,1);

      swarm.follow(waypoints_drone1,waypoints_drone2,setting);

    }

    void on_arm_motors( std_msgs::HeaderConstPtr const & msg ) const {
      ROS_WARN("[rnw_swarm] arm motors of both drones");
      swarm.arm_motors();
    }

    /**
     * Execute the command, return an estimated time cost
     * @param cmd
     * @return estimated duration (sec)
     */
    double execute_rnw_cmd( rnw_command_t const & cmd ) const {

      if ( !swarm.ready() ) {
        ROS_ERROR_STREAM("[rnw_planner] swarm not ready, can not execute commands!");
        return 0;
      }

      ref_frame_t control_frame = calc_control_frame(cmd.setpoint, cmd.heading);

      double rad = deg2rad * 0.5 * rnw_config.swarm.angle;
      Vector3d suspend_pt_drone1 = swarm.drone1.cable_length * Vector3d(0,sin(-rad),cos(-rad));
      Vector3d suspend_pt_drone2 = swarm.drone2.cable_length * Vector3d(0,sin(rad),cos(rad));

      Vector3d setpoint1 = control_frame * suspend_pt_drone1;
      Vector3d setpoint2 = control_frame * suspend_pt_drone2;

      if ( rnw_config.rnw.direct_control ) {
        swarm.send_traj(
                gen_setpoint_traj(swarm.drone1.latest_odom,setpoint1,5),
                gen_setpoint_traj(swarm.drone2.latest_odom,setpoint2,5)
        );
        return 0;
      }
      else {
        quadrotor_msgs::PolynomialTrajectory traj1 = swarm.drone1.plan(setpoint1);
        quadrotor_msgs::PolynomialTrajectory traj2 = swarm.drone2.plan(setpoint2);
        swarm.send_traj(traj1,traj2);
        double dt1 = get_traj_duration(traj1);
        double dt2 = get_traj_duration(traj2);
        ROS_INFO_STREAM("[rnw_planner] execute cmd #" << cmd.seq << ", drone1 " << dt1 << "s, drone2 " << dt2 << "s");
        return std::max(dt1,dt2);
      }

    }

    uint32_t last_cmd_idx = 0;

    /**
     * Spin the rnw_planner.
     * If there is pending command, execute.
     */
    void spin(const ros::TimerEvent &event ){
      rnw_planner.spin();
      if ( !rnw_planner.is_walking ) {
        last_cmd_idx = rnw_planner.cmd.seq;
      }
      else if (rnw_planner.cmd.seq > last_cmd_idx ) {
        ROS_WARN("[swarm_rnw] new command #%u received!",rnw_planner.cmd.seq);
        execute_rnw_cmd(rnw_planner.cmd);
        last_cmd_idx = rnw_planner.cmd.seq;
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

      // settings for initialization
      rnw_config.rnw.phi_epsi = deg2rad * config.phi_epsi;
      rnw_config.rnw.init_threshold = config.init_threshold;
      rnw_config.rnw.init_tau = config.init_tau;
      rnw_config.rnw.init_ang_vel_threshold = config.init_ang_vel_threshold;
      rnw_config.rnw.init_min_step_interval = config.init_min_step_interval;

      // settings for after initialized
      rnw_config.rnw.tau = config.tau;
      rnw_config.rnw.ang_vel_threshold = config.ang_vel_threshold;
      rnw_config.rnw.min_step_interval = config.min_step_interval;

      // settings for energy regulation
      rnw_config.rnw.enable_energy_feedback = config.enable_energy_feedback;
      rnw_config.rnw.EKp = config.EKp;
      rnw_config.rnw.EKi = config.EKi;
      rnw_config.rnw.specify_energy = config.specify_energy;
      rnw_config.rnw.desired_energy = config.desired_energy;

      rnw_config.rnw.desired_nutation = config.desired_nutation;
      rnw_config.rnw.direct_control = config.direct_control;

      rnw_config.rnw.rocking_max_vel = config.rocking_max_vel;
      rnw_config.rnw.rocking_max_acc = config.rocking_max_acc;
      rnw_node.swarm.drone1.set_max_vel_acc(rnw_config.rnw.rocking_max_vel,rnw_config.rnw.rocking_max_acc);
      rnw_node.swarm.drone2.set_max_vel_acc(rnw_config.rnw.rocking_max_vel,rnw_config.rnw.rocking_max_acc);
  });
  server.updateConfig(rnw_config.rnw.to_config());

  ros::spin();

  ros::shutdown();

  return 0;

}