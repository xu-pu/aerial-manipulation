#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <djiros/DroneArmControl.h>
#include <nav_msgs/Odometry.h>

struct arm_manager_node_t {

    bool prepare_to_disarm = false;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_flight_status;
    ros::Subscriber sub_prepare_to_disarm;

    double disarm_height = 0;

    std_msgs::Header latest_disarm_trigger;
    std_msgs::UInt8 latest_flight_status;

    djiros::DroneArmControl arm;

    arm_manager_node_t(){

      ros::NodeHandle nh("~");

      sub_odom = nh.subscribe<nav_msgs::Odometry>(
              "odom",
              1,
              &arm_manager_node_t::on_odom,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_flight_status = nh.subscribe<std_msgs::UInt8>(
              "flight_status",
              1,
              &arm_manager_node_t::on_flight_status,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_prepare_to_disarm = nh.subscribe<std_msgs::Header>(
              "/drone/prepare_to_disarm",
              100,
              &arm_manager_node_t::on_prepare_to_disarm,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_flight_status = nh.subscribe<std_msgs::UInt8>(
              "/djiros/flight_status",
              1,
              &arm_manager_node_t::on_flight_status,
              this,
              ros::TransportHints().tcpNoDelay());
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg ){
      if ( prepare_to_disarm && latest_flight_status.data != 0 && msg->pose.pose.position.z < disarm_height ) {
        ROS_WARN_STREAM("[arm_manager] trying to disarm!!!");
        arm.request.arm = 0;
        ros::service::call("/djiros/drone_arm_control",arm);
      }
      else if ( prepare_to_disarm && latest_flight_status.data == 0 ) {
        ROS_WARN_STREAM("[arm_manager] disarmed");
        prepare_to_disarm = false;
      }
    }

    void on_flight_status( std_msgs::UInt8ConstPtr const & msg ){
      latest_flight_status = *msg;
    }

    void on_prepare_to_disarm( std_msgs::HeaderConstPtr const & msg ){
      latest_disarm_trigger = *msg;
      prepare_to_disarm = true;
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"arm_manager_node");

  arm_manager_node_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}