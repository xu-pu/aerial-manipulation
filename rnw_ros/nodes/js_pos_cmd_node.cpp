#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <quadrotor_msgs/PositionCommand.h>

struct joystick_commander_t {

    ros::NodeHandle & nh;

    nav_msgs::Odometry latest_odom;

    sensor_msgs::Joy latest_joy;

    quadrotor_msgs::PositionCommand latest_cmd;

    ros::Publisher pub_cmd;

    ros::Subscriber sub_joy;

    ros::Subscriber sub_odom;

    ros::Timer timer;

    explicit joystick_commander_t( ros::NodeHandle & _nh ): nh(_nh) {
      pub_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("cmd",100);
      sub_odom = nh.subscribe<nav_msgs::Odometry>(
              "odom",
              10,
              &joystick_commander_t::on_odom,
              this,
              ros::TransportHints().tcpNoDelay()
      );
      sub_joy = nh.subscribe<sensor_msgs::Joy>(
              "joy",
              10,
              &joystick_commander_t::on_js,
              this,
              ros::TransportHints().tcpNoDelay()
      );
      timer = nh.createTimer(ros::Rate(90),&joystick_commander_t::on_timer,this);
    }

    bool js_active() const {
      return true;
    }

    void on_timer( ros::TimerEvent const & e ){
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_odom = *msg;
    }

    void on_js( sensor_msgs::JoyConstPtr const & msg ){
      latest_joy = *msg;
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"js_pos_cmd_node");

  ros::NodeHandle nh("~");

  joystick_commander_t commander(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}