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
#include "rnw_ros/drone_interface.h"
#include "rnw_ros/cone_interface.h"

struct cable_rnw_node_t {

    rnw_config_t & rnw_config;

    rnw_planner_v2_t rnw_planner;

    ros::Timer timer;

    ros::Subscriber sub_trigger_start;

    ros::Subscriber sub_trigger_abort;

    ros::Subscriber sub_trigger_swing_up;

    ros::Subscriber sub_trigger_land;

    ros::Time cmd_start_time;

    ros::Duration cmd_duration;

    drone_interface_t drone;

    cone_interface_t cone;

    ros::Subscriber sub_cone_state;

    cable_rnw_node_t( ros::NodeHandle & nh, rnw_config_t & cfg ) : rnw_config(cfg), rnw_planner(cfg), drone("drone2") {

      drone.set_max_vel_acc(rnw_config.rnw.rocking_max_vel,rnw_config.rnw.max_acc);

      timer = nh.createTimer(ros::Rate(30), &cable_rnw_node_t::spin, this);

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              1,
              &cable_rnw_node_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_trigger_swing_up = nh.subscribe<std_msgs::Header>(
              "/gamepad/Y",
              10,
              &cable_rnw_node_t::on_swing_up,
              this
      );

      sub_trigger_land = nh.subscribe<std_msgs::Header>(
              "/gamepad/A",
              10,
              &cable_rnw_node_t::on_land,
              this
      );

      sub_trigger_start = nh.subscribe<std_msgs::Header>(
              "/gamepad/X",
              10,
              &cable_rnw_node_t::on_start,
              this
      );

      sub_trigger_abort = nh.subscribe<std_msgs::Header>(
              "/abort",
              10,
              &cable_rnw_node_t::on_abort,
              this
      );

    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      rnw_planner.on_cone_state(msg);
    }

    void on_abort( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[cable rnw] abort triggered!");
      rnw_planner.stop_walking();
    }

    void on_swing_up( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[cable rnw] swing up triggered!");

      constexpr int segments = 10;

      vector<Vector3d> waypoints;
      for ( int i=0; i<=segments; i++ ) {
        waypoints.emplace_back(point_at_nutation(
                cone.latest_cone_state,
                uav_utils::from_point_msg(cone.latest_cone_state.tip),
                rnw_config.rnw.desired_nutation * deg2rad * i / segments
        ));
      }

      drone.follow_waypoints(waypoints);

    }

    void on_land( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[cable rnw] land triggered!");

      Vector3d cur_tip = uav_utils::from_point_msg(rnw_planner.latest_cone_state.tip);
      Vector3d cur_contact = uav_utils::from_point_msg(rnw_planner.latest_cone_state.contact_point);

      Vector3d v = cur_tip - cur_contact;
      Vector3d rest_tip = cur_contact + v.norm() * Vector3d(v.x(),v.y(),0).normalized();

      drone.go_to_point(rest_tip);

    }

    void on_start( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[cable rnw] start rnw triggered!");

      if (drone.ready()) {
        rnw_planner.start_walking();
      }
      else {
        ROS_WARN_STREAM("[cable rnw] drone not ready to rnw yet!");
      }

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
        auto traj = drone.gen_traj_go_to_point(cmd.control_point_setpoint);
        drone.execute_trajectory(traj);
        cmd_start_time = ros::Time::now();
        cmd_duration = ros::Duration(get_traj_duration(traj));
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_cable_node");

  ros::NodeHandle nh("~");

  rnw_config_t rnw_config;
  rnw_config.load_from_ros(nh);

  cable_rnw_node_t node(nh, rnw_config);

  ros::spin();

  ros::shutdown();

  return 0;

}