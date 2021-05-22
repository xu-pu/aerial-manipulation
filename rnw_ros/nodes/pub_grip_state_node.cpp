#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/cone_state_estimator.h"

struct grip_state_node_t {

    rnw_config_t rnw_config;

    ros::Publisher pub_grip_state;

    explicit grip_state_node_t( ros::NodeHandle & nh ){
      rnw_config.load_from_ros(nh);
      pub_grip_state = nh.advertise<rnw_msgs::GripState>("/rnw/grip_state",100);
    }

    nav_msgs::Odometry uav_odom;

    bool uav_odom_init = false;

    rnw_msgs::ConeState cone_state;

    bool cone_state_init = false;

    void on_uav_odom( nav_msgs::OdometryConstPtr const & msg ){
      uav_odom = *msg;
      uav_odom_init = true;
    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      cone_state = *msg;
      cone_state_init = true;
    }

    void on_spin() const {
      if ( uav_odom_init && cone_state_init ) {
        grip_state_t grip_state(cone_state, uav_odom, rnw_config.flu_T_tcp);
        pub_grip_state.publish(grip_state.to_msg());
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_grip_state_node");

  ros::NodeHandle nh("~");

  grip_state_node_t node(nh);

  ros::Subscriber sub_uav_odom = nh.subscribe<nav_msgs::Odometry>(
          "/odom/uav",
          10,
          &grip_state_node_t::on_uav_odom,
          &node,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
          "/rnw/cone_state",
          10,
          &grip_state_node_t::on_cone_state,
          &node,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Rate rate(30);

  while ( ros::ok() ) {
    node.on_spin();
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();

  return 0;

}