#include "rnw_ros/traj_uitls.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <nav_msgs/Odometry.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/pose_utils.h"

#include "am_traj/am_traj.hpp"
#include "am_traj/ros_msgs.h"

#include <dynamic_reconfigure/server.h>
#include <rnw_ros/RNWConfig.h>

struct rnw_controller_t {

    ros::Publisher pub_poly_traj;

    nav_msgs::Odometry latest_uav_odom;

    rnw_msgs::ConeState latest_cone_state;

    rnw_config_t rnw_config;

    AmTraj traj_generator;

    AmTraj rocking_generator;

    AmTraj zigzag_generator;

    rnw_planner_t rnw_planner;

    explicit rnw_controller_t(ros::NodeHandle & nh)
            : rnw_planner(nh),
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
    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      latest_cone_state = *msg;
      rnw_planner.on_cone_state(msg);
    }

    void on_trigger_n3ctrl( geometry_msgs::PoseStampedConstPtr const & msg ){}

    void on_trigger_insert( std_msgs::HeaderConstPtr const & msg ){

      ROS_WARN_STREAM("[rnw] insert triggered!");

      Vector3d pt_uav = pose2T(latest_uav_odom.pose.pose);
      Vector3d pt_tip = uav_utils::from_point_msg(latest_cone_state.tip);

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

      ROS_WARN_STREAM("[rnw] topple triggered!");

      Matrix3d R_tip = odom2R(latest_cone_state.odom);
      Vector3d T_tip = ros2eigen(latest_cone_state.tip);
      Vector3d cur_pos = odom2T(latest_uav_odom);

      auto wpts_local = gen_topple_waypoints_local(rnw_config);

      // wpts_local is the desired positions of tcp in the tip frame
      // they need to be transformed to positions of the uav in the world frame
      auto waypoints = transform_pts(transform_pts(wpts_local,R_tip,T_tip),Matrix3d::Identity(),-rnw_config.X_tcp_cage);

      waypoints.insert(waypoints.begin(),cur_pos);

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = traj_generator.genOptimalTrajDTC(waypoints, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));

    }

    void on_rocking(){
      Vector3d tgt_pt = tip_position_to_uav_position(rnw_planner.next_position(),rnw_config);
      Vector3d pt_uav = pose2T(latest_uav_odom.pose.pose);
      Vector3d v0 = Vector3d::Zero();
      // there is a wierd bug, use 3 points will solve it.
      //ROS_INFO_STREAM("[rnw] from " << pt_uav.transpose() << ", to " << tgt_pt.transpose());
      //ROS_INFO_STREAM("[rnw] offset " << (tgt_pt-pt_uav).transpose());
      Vector3d mid_point = (pt_uav+tgt_pt)/2;
      Trajectory traj = rocking_generator.genOptimalTrajDTC({pt_uav, mid_point, tgt_pt}, v0, v0, v0, v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));
      rocking_start_stamp = ros::Time::now();
      rocking_duration = ros::Duration(traj.getTotalDuration());
      is_rocking = true;
    }

    void on_trigger_rnw( std_msgs::HeaderConstPtr const & msg ){
      ROS_WARN_STREAM("[rnw] rnw triggered!");
      rnw_planner.start_planning_cmd();
    }

    void on_trigger_zigzag( std_msgs::HeaderConstPtr const & msg ) const {

      ROS_WARN_STREAM("[rnw] zig-zag triggered!");

      Matrix3d R = ros2eigen(latest_uav_odom.pose.pose.orientation).toRotationMatrix();
      Vector3d T = ros2eigen(latest_uav_odom.pose.pose.position);

      vector<Vector3d> waypoints = gen_waypoint_zigzag(
              rnw_config.zigzag.cycles,
              rnw_config.zigzag.step_forward,
              rnw_config.zigzag.step_sideways
      );

      vector<Vector3d> wps = transform_pts(waypoints,R,T);

      Vector3d v0 = Vector3d::Zero();
      Trajectory traj = zigzag_generator.genOptimalTrajDTC(wps,v0,v0,v0,v0);
      pub_poly_traj.publish(to_ros_msg(traj,ros::Time::now()));

    }

    void cfg_callback( rnw_ros::RNWConfig & config, uint32_t level ){
      ROS_WARN_STREAM("[rnw] re-config rnw");
      rnw_config.rnw.insertion_depth = config.insertion_depth;
      rnw_config.rnw.topple_init = config.topple_init;
      rnw_config.rnw.desired_nutation = config.desired_nutation;
      rnw_config.rnw.tau = config.tau;
      rnw_config.rnw.max_vel = config.max_vel;
      rnw_config.rnw.max_acc = config.max_acc;
      rnw_config.rnw.rocking_max_vel = config.rocking_max_vel;
      rnw_config.rnw.rocking_max_acc = config.rocking_max_acc;
      traj_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.max_vel, rnw_config.rnw.max_acc, 23, 0.02);
      rocking_generator = AmTraj(1024, 16, 0.4, rnw_config.rnw.rocking_max_vel, rnw_config.rnw.rocking_max_acc, 23, 0.02);
      ROS_INFO_STREAM(rnw_config.rnw.to_string());
    }

    bool is_rocking = false;

    ros::Time rocking_start_stamp;
    ros::Duration rocking_duration;

    void on_spin( const ros::TimerEvent &event ){

      if ( is_rocking ) {
        if ( ros::Time::now() > rocking_start_stamp + rocking_duration ) {
          rnw_planner.cmd_ack();
          is_rocking = false;
        }
      }
      else if ( rnw_planner.has_pending_cmd() ) {
        on_rocking();
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_controller_node");

  ros::NodeHandle nh("~");

  rnw_controller_t rnw_controller(nh);

  auto timer = nh.createTimer(ros::Rate(20), &rnw_controller_t::on_spin, &rnw_controller);

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

  ros::Subscriber sub_trigger = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger",10,&rnw_controller_t::on_trigger_n3ctrl,&rnw_controller);

  ros::Subscriber sub_trigger_insert = nh.subscribe<std_msgs::Header>("/rnw/trigger/insert", 10, &rnw_controller_t::on_trigger_insert, &rnw_controller);
  ros::Subscriber sub_trigger_zigzag = nh.subscribe<std_msgs::Header>("/rnw/trigger/zigzag", 10, &rnw_controller_t::on_trigger_zigzag, &rnw_controller);
  ros::Subscriber sub_trigger_topple = nh.subscribe<std_msgs::Header>("/rnw/trigger/topple", 10, &rnw_controller_t::on_trigger_topple, &rnw_controller);
  ros::Subscriber sub_trigger_rnw = nh.subscribe<std_msgs::Header>("/rnw/trigger/rnw", 10, &rnw_controller_t::on_trigger_rnw, &rnw_controller);

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