#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <Eigen/Dense>
#include <uav_utils/geometry_utils.h>
#include <uav_utils/converters.h>
#include <rnw_ros/traj_uitls.h>
#include <am_traj/am_traj.hpp>
#include <am_traj/ros_msgs.h>

using std::vector;
using Eigen::Vector3d;
using Eigen::Matrix3d;

struct test_swarm_planner_t {

    ros::NodeHandle & nh;

    ros::Publisher pub_traj_drone1;

    ros::Publisher pub_traj_drone2;

    bool init_drone1 = false;

    bool init_drone2 = false;

    nav_msgs::Odometry latest_odom_drone1;

    nav_msgs::Odometry latest_odom_drone2;

    AmTraj traj_generator;

    explicit test_swarm_planner_t( ros::NodeHandle & _nh ) : nh(_nh), traj_generator(1024, 16, 0.4, 0.5, 0.5, 23, 0.02) {
      pub_traj_drone1 = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/drone1/traj",10);
      pub_traj_drone2 = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/drone2/traj",10);
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

      if ( !initialized() ) {
        ROS_ERROR_STREAM("[test_swarm_planner_node] did not receive odom");
        return;
      }

      std::vector<Eigen::Vector3d> wpts;
      wpts.emplace_back(0.1,0,0);
      wpts.emplace_back(0.2,0,0);
      wpts.emplace_back(0.3,0,0);
      wpts.emplace_back(0.4,0,0);

      pub_traj_drone1.publish(local_wpts2traj(latest_odom_drone1,wpts));
      pub_traj_drone2.publish(local_wpts2traj(latest_odom_drone2,wpts));

    }

    void trigger_align(){}

    void trigger_circle(){}

    void on_odom_drone1( nav_msgs::OdometryConstPtr const & msg ){
      init_drone1 = true;
      latest_odom_drone1 = *msg;
    }

    void on_odom_drone2( nav_msgs::OdometryConstPtr const & msg ){
      init_drone2 = true;
      latest_odom_drone2 = *msg;
    }

    bool initialized() const {
      return init_drone1 && init_drone2;
    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"test_swarm_planner_node");

  ros::NodeHandle nh("~");

  test_swarm_planner_t planner(nh);

  ros::Subscriber sub_odom_drone1 = nh.subscribe<nav_msgs::Odometry>(
          "/drone1/odom",
          10,
          &test_swarm_planner_t::on_odom_drone1,
          &planner,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_odom_drone2 = nh.subscribe<nav_msgs::Odometry>(
          "/drone1/odom",
          10,
          &test_swarm_planner_t::on_odom_drone2,
          &planner,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_trigger_hello_world = nh.subscribe<std_msgs::Header>(
          "/gamepad/A", 10, &test_swarm_planner_t::trigger_hello_world, &planner);

  ros::spin();

  ros::shutdown();

  return 0;

}
