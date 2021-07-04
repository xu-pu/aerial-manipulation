#include "rnw_ros/drone_interface.h"

#include <Eigen/Dense>
#include <uav_utils/converters.h>
#include <rnw_ros/traj_uitls.h>
#include <am_traj/am_traj.hpp>
#include <am_traj/ros_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <rnw_ros/VelAccConfig.h>

using std::vector;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

double amp = 0.5;
int cycles = 5;

Vector3d point_in_frame( nav_msgs::Odometry const & odom, Vector3d const & pt ){
  Matrix3d R = uav_utils::from_quaternion_msg(odom.pose.pose.orientation).toRotationMatrix();
  Vector3d T = uav_utils::from_point_msg(odom.pose.pose.position);
  return R*pt + T;
}


struct zig_zag_t {

    double interval = 1;

    double acc = 3;

    double vel = 3;

    drone_interface_t & drone;

    double offset = 0.1;

    ros::Timer timer;

    explicit zig_zag_t( drone_interface_t & di ) : drone(di) {}

    void start(){
      drone.set_max_vel_acc(vel,acc);
      ros::NodeHandle nh("~");
      timer = nh.createTimer(interval,&zig_zag_t::on_timer,this);
    }

    void on_timer( ros::TimerEvent const & e ){
      offset = -offset;
      drone.go_to_point_in_intermediate_frame(offset * Vector3d::UnitY());
    }

};

struct individual_drone_test_t {

    ros::NodeHandle & nh;

    drone_interface_t drone;

    ros::Subscriber sub_trigger_zigzag;

    ros::Subscriber sub_trigger_disarm;

    ros::Subscriber sub_trigger_rock;

    ros::Subscriber sub_trigger_take_off;

    AmTraj traj_generator;

    zig_zag_t zig_zag;

    explicit individual_drone_test_t(ros::NodeHandle & _nh ) : nh(_nh), traj_generator(1024, 16, 0.4, 1, 0.5, 23, 0.02),zig_zag(drone){

      drone.init(get_ros_param_required<string>(nh,"drone_name"));

      sub_trigger_zigzag = nh.subscribe<std_msgs::Header>(
              "/gamepad/X", 10, &individual_drone_test_t::trigger_zigzag, this);

      sub_trigger_disarm = nh.subscribe<std_msgs::Header>(
              "/gamepad/A", 10, &individual_drone_test_t::trigger_disarm, this);

      sub_trigger_rock = nh.subscribe<std_msgs::Header>(
              "/gamepad/B", 10, &individual_drone_test_t::trigger_rock, this);

      sub_trigger_take_off = nh.subscribe<std_msgs::Header>(
              "/gamepad/Y", 10, &individual_drone_test_t::trigger_take_off, this);

    }

    /**
     * Waypoints in world frame to Polynomial Trajectory
     * @param odom - local frame
     * @param wpts - waypoints in world frame
     * @return
     */
    quadrotor_msgs::PolynomialTrajectory wpts2traj( nav_msgs::Odometry const & odom, std::vector<Eigen::Vector3d> const & wpts ) const {
      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC(wpts, v0, v0, v0, v0);
      return to_ros_msg(traj,odom,ros::Time::now());
    }

    /**
     * Waypoints in local frame to polynomial trajectory
     * @param odom - local frame
     * @param wpts - waypoints in local frame
     * @return
     */
    quadrotor_msgs::PolynomialTrajectory local_wpts2traj( nav_msgs::Odometry const & odom, vector<Vector3d> const & wpts ) const {
      Matrix3d R = uav_utils::from_quaternion_msg(odom.pose.pose.orientation).toRotationMatrix();
      Vector3d T = uav_utils::from_point_msg(odom.pose.pose.position);
      return wpts2traj(odom,transform_pts(wpts,R,T));
    }

    void trigger_zigzag( std_msgs::HeaderConstPtr const & msg ) {

      if ( !drone.ready(true) ) {
        return;
      }

      zig_zag.start();

    }

    void trigger_disarm( std_msgs::HeaderConstPtr const & msg ) const {

      if ( !drone.ready(true) ) {
        return;
      }

      auto setting = drone_interface_t::create_setting(0.3,0.5);
      Vector3d landing_spot(drone.latest_odom.pose.pose.position.x, drone.latest_odom.pose.pose.position.y, -1);
      drone.execute_trajectory(drone.plan(setting,landing_spot));

    }

    void trigger_rock( std_msgs::HeaderConstPtr const & msg ) const {

      if ( !drone.ready(true) ) {
        return;
      }

      vector<Vector3d> waypoints;
      waypoints.emplace_back(Vector3d::Zero());
      for (int i=0; i < cycles; i++ ) {
        waypoints.emplace_back(0,amp,0);
        waypoints.emplace_back(0,-amp,0);
      }
      waypoints.emplace_back(Vector3d::Zero());

      drone.follow_waypoints_in_intermediate_frame(waypoints);

    }

    void trigger_take_off( std_msgs::HeaderConstPtr const & msg ) const {
      drone.go_to_point_in_intermediate_frame(Vector3d::UnitZ());
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"individual_flight_test_node");

  ros::NodeHandle nh("~");

  individual_drone_test_t node(nh);

  using dyn_cfg_server_t = dynamic_reconfigure::Server<rnw_ros::VelAccConfig>;
  std::shared_ptr<dyn_cfg_server_t> server;
  server = std::make_shared<dyn_cfg_server_t>();
  server->setCallback([&](rnw_ros::VelAccConfig & config, uint32_t level){
      ROS_WARN_STREAM("dynamic reconfigure vel acc limits!");
      node.drone.set_max_vel_acc(config.max_vel,config.max_acc);
      amp = config.amp;
      cycles = config.cycles;
  });

  ros::spin();

  ros::shutdown();

  return 0;

}
