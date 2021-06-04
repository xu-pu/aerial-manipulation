#include "rnw_ros/swarm_interface.h"
#include "rnw_ros/rnw_utils.h"
#include <Eigen/Dense>
#include <uav_utils/converters.h>
#include <rnw_ros/traj_uitls.h>
#include <am_traj/am_traj.hpp>
#include <am_traj/ros_msgs.h>

using std::vector;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;


Vector3d point_in_frame( nav_msgs::Odometry const & odom, Vector3d const & pt ){
  Matrix3d R = uav_utils::from_quaternion_msg(odom.pose.pose.orientation).toRotationMatrix();
  Vector3d T = uav_utils::from_point_msg(odom.pose.pose.position);
  return R*pt + T;
}

struct individual_drone_test_t {

    ros::NodeHandle & nh;

    swarm_interface_t swarm;

    ros::Subscriber sub_trigger_hello_world;
    ros::Subscriber sub_trigger_circle;
    ros::Subscriber sub_trigger_align;
    ros::Subscriber sub_trigger_zigzag;

    ros::Subscriber sub_dpad_up;
    ros::Subscriber sub_dpad_down;
    ros::Subscriber sub_dpad_left;
    ros::Subscriber sub_dpad_right;

    AmTraj traj_generator;

    explicit individual_drone_test_t(ros::NodeHandle & _nh ) : nh(_nh), swarm(_nh), traj_generator(1024, 16, 0.4, 1, 0.5, 23, 0.02) {

      sub_trigger_hello_world = nh.subscribe<std_msgs::Header>(
              "/gamepad/A", 10, &individual_drone_test_t::trigger_hello_world, this);

      sub_trigger_circle = nh.subscribe<std_msgs::Header>(
              "/gamepad/B", 10, &individual_drone_test_t::trigger_circle, this);

      sub_trigger_align = nh.subscribe<std_msgs::Header>(
              "/gamepad/Y", 10, &individual_drone_test_t::trigger_align, this);

      sub_trigger_zigzag = nh.subscribe<std_msgs::Header>(
              "/gamepad/X", 10, &individual_drone_test_t::trigger_zigzag, this);

      sub_dpad_up = nh.subscribe<std_msgs::Header>(
              "/gamepad/DPAD/Up", 10, &individual_drone_test_t::on_dpad_up, this);

      sub_dpad_down = nh.subscribe<std_msgs::Header>(
              "/gamepad/DPAD/Down", 10, &individual_drone_test_t::on_dpad_down, this);

      sub_dpad_left = nh.subscribe<std_msgs::Header>(
              "/gamepad/DPAD/Left", 10, &individual_drone_test_t::on_dpad_left, this);

      sub_dpad_right = nh.subscribe<std_msgs::Header>(
              "/gamepad/DPAD/Right", 10, &individual_drone_test_t::on_dpad_right, this);

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

    void trigger_hello_world( std_msgs::HeaderConstPtr const & msg ) const {

      if ( !swarm.ready() ) {
        ROS_ERROR_STREAM("[test_swarm_planner_node] swarm not ready");
        return;
      }

      std::vector<Eigen::Vector3d> wpts;
      wpts.emplace_back(0,0,0);
      wpts.emplace_back(0.1,0,0.1);
      wpts.emplace_back(0.2,0,0.15);
      wpts.emplace_back(0.3,0,0.2);
      wpts.emplace_back(0.4,0,0.2);

      swarm.send_traj(
              local_wpts2traj(swarm.drone1.latest_odom,wpts),
              local_wpts2traj(swarm.drone2.latest_odom,wpts)
      );

    }

    void trigger_align( std_msgs::HeaderConstPtr const & msg ) const {

      if ( !swarm.ready() ) {
        ROS_ERROR_STREAM("[test_swarm_planner_node] swarm not ready");
        return;
      }

      Vector3d pt_srt = uav_utils::from_point_msg(swarm.drone1.latest_odom.pose.pose.position);
      Vector3d pt_end = point_in_frame(swarm.drone2.latest_odom,Vector3d(0,1.4,0));

      if ((pt_srt-pt_end).norm() < 0.05) {
        ROS_WARN_STREAM("close enough, no need to align");
        return;
      }

      vector<Vector3d> wpts;
      wpts.push_back(pt_srt);
      wpts.emplace_back((pt_srt+pt_end)/2);
      wpts.push_back(pt_end);

      swarm.send_traj_just_drone1(wpts2traj(swarm.drone1.latest_odom,wpts));

    }

    void trigger_zigzag( std_msgs::HeaderConstPtr const & msg ) const {

      if ( !swarm.ready() ) {
        ROS_ERROR_STREAM("[test_swarm_planner_node] swarm not ready");
        return;
      }

      vector<Vector3d> wpts = gen_waypoint_zigzag(5,0.5,0.5);

      swarm.send_traj(
              local_wpts2traj(swarm.drone1.latest_odom,wpts),
              local_wpts2traj(swarm.drone2.latest_odom,wpts)
      );

    }

    void trigger_circle( std_msgs::HeaderConstPtr const & msg ) const {

      if ( !swarm.ready() ) {
        ROS_ERROR_STREAM("[test_swarm_planner_node] swarm not ready!");
        return;
      }

      Vector3d pos1 = uav_utils::from_point_msg(swarm.drone1.latest_odom.pose.pose.position);
      Vector3d pos2 = uav_utils::from_point_msg(swarm.drone2.latest_odom.pose.pose.position);

      double dist = (pos1 - pos2).norm();

      if ( dist < 0.5 ) {
        ROS_ERROR_STREAM("[test_swarm_planner_node] drones too close");
        return;
      }

      Vector2d p1(pos1.x(),pos1.y());
      Vector2d p2(pos2.x(),pos2.y());

      Vector2d mid = (p1+p2)/2;

      Vector2d v1 = p1-mid;
      Vector2d v2 = p2-mid;

      int segments = 12;
      double interval = M_PI*2/segments;

      vector<Vector3d> wpts_drone1;
      vector<Vector3d> wpts_drone2;
      for ( int i=0; i<=segments; i++ ) {
        Eigen::Rotation2Dd rot(interval*i);
        Vector2d pt1 = mid + (rot * v1);
        wpts_drone1.emplace_back(pt1.x(), pt1.y(), pos1.z());
        Vector2d pt2 = mid + (rot * v2);
        wpts_drone2.emplace_back(pt2.x(), pt2.y(), pos2.z());
      }

      swarm.send_traj(
              wpts2traj(swarm.drone1.latest_odom,wpts_drone1),
              wpts2traj(swarm.drone2.latest_odom,wpts_drone2)
      );

    }

    void move_drone1_relative( Vector3d const & pt ){
      Vector3d setpoint = uav_utils::from_point_msg(swarm.drone1.latest_odom.pose.pose.position) + 0.5 * pt;
      quadrotor_msgs::PolynomialTrajectory traj = gen_setpoint_traj(swarm.drone1.latest_odom, setpoint, 1);
      swarm.send_traj_just_drone1(traj);
    }

    void on_dpad_up( std_msgs::HeaderConstPtr const & msg ){
      move_drone1_relative({ 1, 0, 0 });
    }

    void on_dpad_down( std_msgs::HeaderConstPtr const & msg ){
      move_drone1_relative({ -1, 0, 0 });
    }

    void on_dpad_left( std_msgs::HeaderConstPtr const & msg ){
      move_drone1_relative({ 0, 1, 0 });
    }

    void on_dpad_right( std_msgs::HeaderConstPtr const & msg ){
      move_drone1_relative({ 0, -1, 0 });
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"test_swarm_planner_node");

  ros::NodeHandle nh("~");

  individual_drone_test_t planner(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}
