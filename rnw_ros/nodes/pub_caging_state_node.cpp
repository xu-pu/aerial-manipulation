#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"

#include "rnw_ros/drone_interface.h"
#include "rnw_ros/cone_interface.h"
#include "rnw_ros/caging_state.h"

struct grip_state_node_t {

    rnw_config_t rnw_config;

    ros::Publisher pub_grip_state;

    drone_interface_t drone;

    cone_interface_t cone;

    ros::Timer timer;

    explicit grip_state_node_t(): drone("drone1") { // only drone1 can do caging

      ros::NodeHandle nh("~");

      rnw_config.load_from_ros(nh);

      pub_grip_state = nh.advertise<rnw_msgs::GripState>("/rnw/caging",100);

      timer = nh.createTimer(ros::Rate(30),&grip_state_node_t::on_spin,this);

    }

    void on_spin( ros::TimerEvent const & e ) const {
      if (drone.odom_in_time() && cone.odom_in_time() ) {
        pub_grip_state.publish(caging_state_t(cone.latest_cone_state,drone.latest_odom,rnw_config.flu_T_tcp).to_msg());
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_caging_state_node");

  grip_state_node_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}