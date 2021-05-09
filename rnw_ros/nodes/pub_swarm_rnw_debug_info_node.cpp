#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_msgs/ConeState.h"

struct swarm_rnw_debugger_t {

    ros::Subscriber sub_cone_state;

    ros::Subscriber sub_odom_drone1;

    ros::Subscriber sub_odom_drone2;

    ros::Publisher pub_nutation;

    ros::Publisher pub_cable_drone1;

    ros::Publisher pub_cable_drone2;

    nav_msgs::Odometry latest_odom_drone1;

    nav_msgs::Odometry latest_odom_drone2;

    rnw_msgs::ConeState latest_cone_state;

    swarm_rnw_debugger_t(){

      ros::NodeHandle nh("~");

      pub_cable_drone1 = nh.advertise<quadrotor_msgs::Float64Stamped>("/drone1/cable",100);
      pub_cable_drone2 = nh.advertise<quadrotor_msgs::Float64Stamped>("/drone2/cable",100);
      pub_nutation = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/nutation_deg",100);

      sub_odom_drone1 = nh.subscribe<nav_msgs::Odometry>(
              "/drone1/odom",
              1,
              &swarm_rnw_debugger_t::on_odom_drone1,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_odom_drone2 = nh.subscribe<nav_msgs::Odometry>(
              "/drone2/odom",
              1,
              &swarm_rnw_debugger_t::on_odom_drone2,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              1,
              &swarm_rnw_debugger_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

    }

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){
      latest_cone_state = *msg;

      quadrotor_msgs::Float64Stamped rst;
      rst.header = msg->header;

      rst.value = latest_cone_state.euler_angles.y * rad2deg;
      pub_nutation.publish(rst);

      rst.value = ( uav_utils::from_point_msg(latest_odom_drone1.pose.pose.position) - uav_utils::from_point_msg(latest_cone_state.tip) ).norm();
      pub_cable_drone1.publish(rst);

      rst.value = ( uav_utils::from_point_msg(latest_odom_drone2.pose.pose.position) - uav_utils::from_point_msg(latest_cone_state.tip) ).norm();
      pub_cable_drone2.publish(rst);

    }

    void on_odom_drone1( nav_msgs::OdometryConstPtr const & msg ){
      latest_odom_drone1 = *msg;
    }

    void on_odom_drone2( nav_msgs::OdometryConstPtr const & msg ){
      latest_odom_drone2 = *msg;
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_swarm_rnw_debug_info_node");

  ros::NodeHandle nh("~");

  swarm_rnw_debugger_t swarm_rnw_debugger;

  ros::spin();

  ros::shutdown();

  return 0;

}