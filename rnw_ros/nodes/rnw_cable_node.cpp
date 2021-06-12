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

    drone_interface_t drone;

    cone_interface_t cone;

    ros::Subscriber sub_cone_state;

    cable_rnw_node_t( ros::NodeHandle & nh, rnw_config_t & cfg ) : rnw_config(cfg), rnw_planner(cfg) {

      drone.init(get_ros_param_required<string>(nh,"drone_name"));

      drone.set_max_vel_acc(rnw_config.rnw.rocking_max_vel,rnw_config.rnw.rocking_max_acc);

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

      Vector3d suspend_pt = Vector3d::UnitZ() * drone.cable_length;

      double cur_nutation = rad2deg * cone.latest_cone_state.euler_angles.y;

      vector<Vector3d> waypoints;
      for ( double theta : range(cur_nutation,rnw_config.rnw.desired_nutation,10) ) {
        Vector3d tip = cone.tip_at_nutation(theta * deg2rad);
        waypoints.emplace_back(tip+suspend_pt);
      }

      drone.follow_waypoints(waypoints);

    }

    void on_land( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[cable rnw] land triggered!");

      Vector3d C = cone.contact_point();
      Vector3d rest_tip = cone.tip_at_rest();
      Vector3d dir = (rest_tip - C).normalized();
      Vector3d landing_spot = rest_tip + 0.6 * drone.cable_length * dir;
      Vector3d stopping_spot = landing_spot;
      stopping_spot.z() = -0.5;

      vector<Vector3d> waypoints;
      if ( cone.latest_cone_state.euler_angles.y < 75 * deg2rad ) {
        waypoints.emplace_back(rest_tip + drone.cable_length * Vector3d::UnitZ());
      }
      waypoints.emplace_back(landing_spot);
      waypoints.emplace_back(stopping_spot);

      auto setting = drone_interface_t::create_setting(0.5,0.5);

      drone.execute_trajectory(drone.plan(setting,waypoints));

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

    uint32_t last_cmd_idx = 0;

    void spin(const ros::TimerEvent &event ){
      rnw_planner.spin();
      if ( !rnw_planner.is_walking ) {
        last_cmd_idx = rnw_planner.cmd.seq;
      }
      else if (rnw_planner.cmd.seq > last_cmd_idx ) {
        ROS_WARN("[swarm_rnw] new command #%u received!",rnw_planner.cmd.seq);
        drone.execute_trajectory(
                drone.plan(rnw_planner.cmd.setpoint + drone.cable_length * Vector3d::UnitZ())
        );
        last_cmd_idx = rnw_planner.cmd.seq;
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