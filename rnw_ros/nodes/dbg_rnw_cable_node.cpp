#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"

#include "rnw_ros/drone_interface.h"
#include "rnw_ros/cone_interface.h"
#include "rnw_ros/caging_state.h"

struct dbg_rnw_cable_node_t {

    rnw_config_t rnw_config;

    ros::Publisher pub_flu_cp_dist;

    drone_interface_t drone;

    cone_interface_t cone;

    ros::Timer timer;

    explicit dbg_rnw_cable_node_t(): drone("drone2") { // only drone1 can do caging

      ros::NodeHandle nh("~");

      rnw_config.load_from_ros(nh);

      pub_flu_cp_dist = nh.advertise<quadrotor_msgs::Float64Stamped>("/rnw/dist_flu_cp",100);

      timer = nh.createTimer(ros::Rate(30),&dbg_rnw_cable_node_t::on_spin,this);

    }

    void on_spin( ros::TimerEvent const & e ) const {
      if (drone.odom_in_time() && cone.odom_in_time() ) {
        quadrotor_msgs::Float64Stamped msg;
        msg.header = drone.latest_odom.header;
        msg.value = (uav_utils::from_point_msg(drone.latest_odom.pose.pose.position) - uav_utils::from_point_msg(cone.latest_cone_state.tip)).norm();
        pub_flu_cp_dist.publish(msg);
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"dbg_rnw_cable_node");

  dbg_rnw_cable_node_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}