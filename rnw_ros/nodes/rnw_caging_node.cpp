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

struct caging_rnw_node_t {

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

    caging_rnw_node_t( ros::NodeHandle & nh, rnw_config_t & cfg ) : rnw_config(cfg), rnw_planner(cfg), drone("drone1") {

      drone.set_max_vel_acc(rnw_config.rnw.rocking_max_vel,rnw_config.rnw.rocking_max_acc);

      timer = nh.createTimer(ros::Rate(30), &caging_rnw_node_t::spin, this);

      sub_trigger_swing_up = nh.subscribe<std_msgs::Header>(
              "/gamepad/Y",
              10,
              &caging_rnw_node_t::on_topple,
              this
      );

      sub_trigger_abort = nh.subscribe<std_msgs::Header>(
              "/abort",
              10,
              &caging_rnw_node_t::on_abort,
              this
      );

    }

    void on_abort( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw_planner] abort triggered!");
      rnw_planner.stop_walking();
    }

    void on_topple( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[caging rnw] topple triggered triggered!");

      auto waypoints = gen_wpts_push_topple(drone.latest_odom,cone.latest_cone_state,rnw_config);

      drone.follow_waypoints(waypoints,uav_yaw_from_cone_state(cone.latest_cone_state));

    }

    /**
     * Spin the rnw_planner.
     * If there is pending command, execute.
     * After execution, rnw_planner.cmd_complete()
     */
    void spin(const ros::TimerEvent &event ){}

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_caging_node");

  ros::NodeHandle nh("~");

  rnw_config_t rnw_config;
  rnw_config.load_from_ros(nh);

  caging_rnw_node_t node(nh, rnw_config);

  ros::spin();

  ros::shutdown();

  return 0;

}