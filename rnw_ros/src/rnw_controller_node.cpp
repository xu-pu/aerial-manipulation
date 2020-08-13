#include "rnw_ros/traj_uitls.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <nav_msgs/Odometry.h>

#include "am_traj/am_traj.hpp"
#include "am_traj/ros_msgs.h"

struct rnw_controller_t {

    ros::Publisher pub_poly_traj;

    nav_msgs::Odometry latest_uav_odom;

    geometry_msgs::PointStamped latest_cone_tip;

    explicit rnw_controller_t(ros::NodeHandle & nh){
      pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("poly_traj",10,true);
    }

    void on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_uav_odom = *msg;
    }

    void on_cone_tip( geometry_msgs::PointStampedConstPtr const & msg ){
      latest_cone_tip = *msg;
    }

    void on_trigger_n3ctrl( geometry_msgs::PoseStampedConstPtr const & msg ){}

    void on_trigger_insert( std_msgs::HeaderConstPtr const & msg ){}

    void on_trigger_go_to_tip( std_msgs::HeaderConstPtr const & msg ){}

    void on_trigger_rock( std_msgs::HeaderConstPtr const & msg ){}

};

int main( int argc, char** argv ) {

  sleep(3);

  ros::init(argc,argv,"rnw_controller_node");

  ros::NodeHandle nh("~");

  rnw_controller_t rnw_controller(nh);

  ros::Subscriber sub_uav_odom = nh.subscribe<nav_msgs::Odometry>("/uwb_vicon_odom",10,&rnw_controller_t::on_uav_odom,&rnw_controller);
  ros::Subscriber sub_cone_tip = nh.subscribe<geometry_msgs::PointStamped>("/cone/tip",1,&rnw_controller_t::on_cone_tip,&rnw_controller);

  ros::Subscriber sub_trigger = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger",10,&rnw_controller_t::on_trigger_n3ctrl,&rnw_controller);

  ros::Subscriber sub_trigger_tip = nh.subscribe<std_msgs::Header>("/rnw/trigger/go_to_tip", 10, &rnw_controller_t::on_trigger_go_to_tip, &rnw_controller);
  ros::Subscriber sub_trigger_insert = nh.subscribe<std_msgs::Header>("/rnw/trigger/insert", 10, &rnw_controller_t::on_trigger_insert, &rnw_controller);
  ros::Subscriber sub_trigger_rock = nh.subscribe<std_msgs::Header>("/rnw/trigger/rock", 10, &rnw_controller_t::on_trigger_rock, &rnw_controller);

  ros::spin();

  ros::shutdown();

  return 0;

}