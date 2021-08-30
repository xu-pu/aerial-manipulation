#include "ros/ros.h"
#include "rnw_ros/cone_interface.h"
#include "rnw_ros/drone_interface.h"
#include "rnw_ros/rnw_utils.h"

#include <sensor_msgs/Joy.h>

struct caging_rl_t {

    cone_interface_t cone;

    drone_interface_t drone;

    rnw_config_t config;

    ros::Subscriber sub_gamepad_Y;

    ros::Subscriber sub_gamepad_X;

    ros::Subscriber sub_gamepad_RB;

    caging_rl_t(): drone("drone1") {

      ros::NodeHandle nh("~");

      config.load_from_ros(nh);

      sub_gamepad_Y = nh.subscribe<std_msgs::Header>("/gamepad/Y",10,&caging_rl_t::on_topple,this);

    }

    void on_topple( std_msgs::HeaderConstPtr const & msg ) const {

      ROS_WARN("[caging] topple triggered!");

      vector<Vector3d> tip_wpts;

      double cur_nutation = rad2deg * cone.latest_cone_state.euler_angles.y;

      for ( double theta : range(cur_nutation,config.rnw.desired_nutation,5) ) {
        tip_wpts.emplace_back( cone.tip_at_nutation(deg2rad*theta) );
      }

      // build waypoints
      vector<Vector3d> drone_wpts;
      drone_wpts.emplace_back(drone.position());
      for ( auto const pt : tip_wpts ) {
        drone_wpts.emplace_back( pt - config.flu_T_tcp );
      }

      drone.follow_waypoints(drone_wpts, cone_yaw(cone.latest_cone_state));

    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"caging_rl_node");

  caging_rl_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}