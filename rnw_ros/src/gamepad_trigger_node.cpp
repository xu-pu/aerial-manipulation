#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "rnw_ros/ros_utils.h"

struct gamepad_handler_t {

    ros::Publisher pub_A;

    ros::Publisher pub_B;

    ros::Publisher pub_X;

    ros::Publisher pub_Y;

    sensor_msgs::Joy latest_joy;

    bool init = false;

    static constexpr size_t BUTTON_X = 0;
    static constexpr size_t BUTTON_A = 1;
    static constexpr size_t BUTTON_B = 2;
    static constexpr size_t BUTTON_Y = 3;

    explicit gamepad_handler_t( ros::NodeHandle & nh ){
      pub_A = nh.advertise<std_msgs::Header>("A",10,false);
      pub_B = nh.advertise<std_msgs::Header>("B",10,false);
      pub_X = nh.advertise<std_msgs::Header>("X",10,false);
      pub_Y = nh.advertise<std_msgs::Header>("Y",10,false);
    }

    void on_joystick( sensor_msgs::JoyConstPtr const & msg ){

      if ( !init ) {
        init = true;
        latest_joy = *msg;
        return;
      }

      sensor_msgs::Joy pre_msg = latest_joy;
      sensor_msgs::Joy cur_msg = *msg;
      latest_joy = *msg;

      if ( pre_msg.buttons.at(BUTTON_A) > cur_msg.buttons.at(BUTTON_A) ) {
        ROS_INFO_STREAM("Button A triggered!");
        pub_A.publish(std_msgs::Header());
      }

      if ( pre_msg.buttons.at(BUTTON_B) > cur_msg.buttons.at(BUTTON_B) ) {
        ROS_INFO_STREAM("Button B triggered!");
        pub_B.publish(std_msgs::Header());
      }

      if ( pre_msg.buttons.at(BUTTON_X) > cur_msg.buttons.at(BUTTON_X) ) {
        ROS_INFO_STREAM("Button X triggered!");
        pub_X.publish(std_msgs::Header());
      }

      if ( pre_msg.buttons.at(BUTTON_Y) > cur_msg.buttons.at(BUTTON_Y) ) {
        ROS_INFO_STREAM("Button Y triggered!");
        pub_Y.publish(std_msgs::Header());
      }

    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"gamepad_trigger_node");

  ros::NodeHandle nh("~");

  gamepad_handler_t gamepad_handler(nh);

  ros::Subscriber sub_odom = nh.subscribe<sensor_msgs::Joy>("/joy",100,&gamepad_handler_t::on_joystick,&gamepad_handler);

  ros::spin();

  ros::shutdown();

  return 0;

}