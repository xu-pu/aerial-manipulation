#include "rnw_ros/traj_uitls.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <nav_msgs/Odometry.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/pose_utils.h"

#include "am_traj/am_traj.hpp"
#include "am_traj/ros_msgs.h"

struct rnw_controller_t {

    ros::Publisher pub_poly_traj;

    nav_msgs::Odometry latest_uav_odom;

    nav_msgs::Odometry latest_cone_odom;

    geometry_msgs::PointStamped latest_cone_tip;

    rnw_config_t rnw_config;

    AmTraj traj_generator;

    explicit rnw_controller_t(ros::NodeHandle & nh)
            : traj_generator(1024, 16, 0.4, 1, 0.5, 23, 0.02)
    {
      pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("poly_traj",10,true);
      rnw_config.load_from_ros(nh);
    }

    void on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_uav_odom = *msg;
    }

    void on_cone_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_cone_odom = *msg;
    }

    void on_cone_tip( geometry_msgs::PointStampedConstPtr const & msg ){
      latest_cone_tip = *msg;
    }

    void on_trigger_n3ctrl( geometry_msgs::PoseStampedConstPtr const & msg ){}

    void on_trigger_insert( std_msgs::HeaderConstPtr const & msg ){

      Vector3d pt_uav = pose2T(latest_uav_odom.pose.pose);
      Vector3d pt_tip(latest_cone_tip.point.x, latest_cone_tip.point.y, latest_cone_tip.point.z);

      double z_planned_tcp = pt_tip.z() + rnw_config.hover_above_tip;
      double z_planned_uav = z_planned_tcp - rnw_config.X_tcp_cage.z(); // offset between imu and tcp

      Vector3d pt_tgt = pt_tip;
      pt_tgt.z() = z_planned_uav;

      Vector3d pt_inserted = pt_tgt;
      pt_inserted.z()  = pt_inserted.z() - rnw_config.hover_above_tip - rnw_config.insert_below_tip;

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC({pt_uav, pt_tgt, pt_inserted}, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));

    }

    void on_trigger_topple( std_msgs::HeaderConstPtr const & msg ) const {

      Matrix3d R_tip = odom2R(latest_cone_odom);
      Vector3d T_tip = ros2eigen(latest_cone_tip.point);
      Vector3d cur_pos = odom2T(latest_uav_odom);

      auto wpts_local = gen_topple_waypoints_local(rnw_config.hover_above_tip,rnw_config.insert_below_tip);
      auto waypoints = transform_pts(wpts_local,R_tip,T_tip);
      waypoints.insert(waypoints.begin(),cur_pos);

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));

    }

    void on_trigger_go_to_tip( std_msgs::HeaderConstPtr const & msg ){

      Vector3d pt_uav = pose2T(latest_uav_odom.pose.pose);
      Vector3d pt_tip(latest_cone_tip.point.x, latest_cone_tip.point.y, latest_cone_tip.point.z);

      double z_planned_tcp = pt_tip.z() + rnw_config.hover_above_tip;
      double z_planned_uav = z_planned_tcp - rnw_config.X_tcp_cage.z(); // offset between imu and tcp

      Vector3d pt_tgt = pt_tip;
      pt_tgt.z() = z_planned_uav;

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC({pt_uav, pt_tgt}, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));

    }

    void on_trigger_rock( std_msgs::HeaderConstPtr const & msg ){}

    void on_trigger_zigzag( std_msgs::HeaderConstPtr const & msg ) const {

      Matrix3d R = ros2eigen(latest_uav_odom.pose.pose.orientation).toRotationMatrix();
      Vector3d T = ros2eigen(latest_uav_odom.pose.pose.position);

      vector<Vector3d> waypoints = gen_waypoint_zigzag(5,0.1,0.2);
      vector<Vector3d> wps = transform_pts(waypoints,R,T);

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC(wps,v0,v0,v0,v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));

    }

};

int main( int argc, char** argv ) {

  sleep(3);

  ros::init(argc,argv,"rnw_controller_node");

  ros::NodeHandle nh("~");

  rnw_controller_t rnw_controller(nh);

  ros::Subscriber sub_uav_odom = nh.subscribe<nav_msgs::Odometry>("/uwb_vicon_odom",10,&rnw_controller_t::on_uav_odom,&rnw_controller);
  ros::Subscriber sub_cone_odom = nh.subscribe<nav_msgs::Odometry>("/uwb_vicon_odom_cone",10,&rnw_controller_t::on_cone_odom,&rnw_controller);
  ros::Subscriber sub_cone_tip = nh.subscribe<geometry_msgs::PointStamped>("/cone/tip",1,&rnw_controller_t::on_cone_tip,&rnw_controller);

  ros::Subscriber sub_trigger = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger",10,&rnw_controller_t::on_trigger_n3ctrl,&rnw_controller);

  ros::Subscriber sub_trigger_tip = nh.subscribe<std_msgs::Header>("trigger_go_to_tip", 10, &rnw_controller_t::on_trigger_go_to_tip, &rnw_controller);
  ros::Subscriber sub_trigger_insert = nh.subscribe<std_msgs::Header>("trigger_insert", 10, &rnw_controller_t::on_trigger_insert, &rnw_controller);
  ros::Subscriber sub_trigger_rock = nh.subscribe<std_msgs::Header>("trigger_rock", 10, &rnw_controller_t::on_trigger_rock, &rnw_controller);
  ros::Subscriber sub_trigger_zigzag = nh.subscribe<std_msgs::Header>("trigger_zigzag", 10, &rnw_controller_t::on_trigger_zigzag, &rnw_controller);
  ros::Subscriber sub_trigger_topple = nh.subscribe<std_msgs::Header>("trigger_topple", 10, &rnw_controller_t::on_trigger_topple, &rnw_controller);

  ros::spin();

  ros::shutdown();

  return 0;

}