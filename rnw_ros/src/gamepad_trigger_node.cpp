#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

void on_joystick( sensor_msgs::JoyConstPtr const & msg ){
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"gamepad_trigger_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_odom = nh.subscribe<sensor_msgs::Joy>("/joy",100,on_joystick);

  ros::spin();

  ros::shutdown();

  return 0;

}