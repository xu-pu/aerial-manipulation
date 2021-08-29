#include "ros/ros.h"
#include "rnw_ros/cone_interface.h"
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/drone_interface.h"

#include <sensor_msgs/Joy.h>

struct vel_commander_t {

    drone_interface_t drone;

    ros::Timer command_loop;

    Vector3d cmd_vel;

    ros::Time start_time;

    ros::Subscriber sub_trigger_start;

    ros::Subscriber sub_trigger_stop;

    vel_commander_t(): drone("drone1") {

      ros::NodeHandle nh("~");

      start_time = ros::Time::now();

      sub_trigger_start = nh.subscribe<std_msgs::Header>(
              "/gamepad/X",
              10,
              &vel_commander_t::on_start,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_trigger_stop = nh.subscribe<std_msgs::Header>(
              "/gamepad/RB",
              10,
              &vel_commander_t::on_stop,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      command_loop = nh.createTimer(
              ros::Rate(70),
              &vel_commander_t::on_command_loop,
              this,
              false,
              false
      );

    }

    void on_start( std_msgs::HeaderConstPtr const & msg ){
      ROS_INFO("start command loop");
      command_loop.start();
    }

    void on_stop( std_msgs::HeaderConstPtr const & msg ){
      ROS_INFO("stop command loop");
      command_loop.stop();
    }

    void on_command_loop( ros::TimerEvent const & e ){
      double span = 1;
      double sec_since_boot = ( ros::Time::now() - start_time ).toSec();
      double osc_val = sin(span*sec_since_boot);
      cmd_vel = osc_val * Vector3d::UnitX();
      drone.cmd_pos_vel(drone.position(), cmd_vel);
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"test_vel_cmd_node");

  vel_commander_t commander;

  ros::spin();

  ros::shutdown();

  return 0;

}