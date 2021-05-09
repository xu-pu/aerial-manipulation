#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "rnw_ros/ros_utils.h"

struct gamepad_handler_t {

    ros::Publisher pub_A;

    ros::Publisher pub_B;

    ros::Publisher pub_X;

    ros::Publisher pub_Y;

    ros::Publisher pub_RB;

    ros::Publisher pub_LB;

    ros::Publisher pub_RT;

    ros::Publisher pub_LT;

    ros::Publisher pub_dpad_up;

    ros::Publisher pub_dpad_down;

    ros::Publisher pub_dpad_left;

    ros::Publisher pub_dpad_right;

    sensor_msgs::Joy latest_joy;

    bool init = false;

    static constexpr size_t BUTTON_X = 2;
    static constexpr size_t BUTTON_A = 0;
    static constexpr size_t BUTTON_B = 1;
    static constexpr size_t BUTTON_Y = 3;
    static constexpr size_t BUTTON_LB = 4;
    static constexpr size_t BUTTON_RB = 5;
    static constexpr size_t BUTTON_LT = 6;
    static constexpr size_t BUTTON_RT = 7;

    static constexpr size_t DPAD_UP_DOWN = 7;
    static constexpr size_t DPAD_LEFT_RIGHT = 6;

    explicit gamepad_handler_t( ros::NodeHandle & nh ){
      pub_A = nh.advertise<std_msgs::Header>("A",10,false);
      pub_B = nh.advertise<std_msgs::Header>("B",10,false);
      pub_X = nh.advertise<std_msgs::Header>("X",10,false);
      pub_Y = nh.advertise<std_msgs::Header>("Y",10,false);
      pub_RB = nh.advertise<std_msgs::Header>("RB",10,false);
      pub_LB = nh.advertise<std_msgs::Header>("LB",10,false);
      pub_RT = nh.advertise<std_msgs::Header>("RT",10,false);
      pub_LT = nh.advertise<std_msgs::Header>("LT",10,false);
      pub_dpad_up    = nh.advertise<std_msgs::Header>("DPAD/Up",10,false);
      pub_dpad_down  = nh.advertise<std_msgs::Header>("DPAD/Down",10,false);
      pub_dpad_left  = nh.advertise<std_msgs::Header>("DPAD/Left",10,false);
      pub_dpad_right = nh.advertise<std_msgs::Header>("DPAD/Right",10,false);
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

      if ( pre_msg.buttons.at(BUTTON_LB) > cur_msg.buttons.at(BUTTON_LB) ) {
        ROS_INFO_STREAM("Button LB triggered!");
        pub_LB.publish(std_msgs::Header());
      }

      if ( pre_msg.buttons.at(BUTTON_RB) > cur_msg.buttons.at(BUTTON_RB) ) {
        ROS_INFO_STREAM("Button RB triggered!");
        pub_RB.publish(std_msgs::Header());
      }

      if ( pre_msg.buttons.at(BUTTON_LT) > cur_msg.buttons.at(BUTTON_LT) ) {
        ROS_INFO_STREAM("Button LT triggered!");
        pub_LT.publish(std_msgs::Header());
      }

      if ( pre_msg.buttons.at(BUTTON_RT) > cur_msg.buttons.at(BUTTON_RT) ) {
        ROS_INFO_STREAM("Button RT triggered!");
        pub_RT.publish(std_msgs::Header());
      }

      int pre_dpad_v = static_cast<int>(pre_msg.axes.at(DPAD_UP_DOWN));
      int cur_dpad_v = static_cast<int>(cur_msg.axes.at(DPAD_UP_DOWN));

      int pre_dpad_h = static_cast<int>(pre_msg.axes.at(DPAD_LEFT_RIGHT));
      int cur_dpad_h = static_cast<int>(cur_msg.axes.at(DPAD_LEFT_RIGHT));

      if ( pre_dpad_h != cur_dpad_h && cur_dpad_h == 1 ) {
        ROS_INFO_STREAM("DPAD Left triggered!");
        pub_dpad_left.publish(std_msgs::Header());
      }

      if ( pre_dpad_h != cur_dpad_h && cur_dpad_h == -1 ) {
        ROS_INFO_STREAM("DPAD Right triggered!");
        pub_dpad_right.publish(std_msgs::Header());
      }

      if ( pre_dpad_v != cur_dpad_v && cur_dpad_v == 1 ) {
        ROS_INFO_STREAM("DPAD Up triggered!");
        pub_dpad_up.publish(std_msgs::Header());
      }

      if ( pre_dpad_v != cur_dpad_v && cur_dpad_v == -1 ) {
        ROS_INFO_STREAM("DPAD Down triggered!");
        pub_dpad_down.publish(std_msgs::Header());
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