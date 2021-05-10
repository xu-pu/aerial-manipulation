#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "rnw_ros/rnw_utils.h"

struct joystick_commander_t {

    static constexpr double js_epsi = 0.01;

    static constexpr double scale_vx = 0.3;

    static constexpr double scale_vy = 0.3;

    static constexpr double scale_vz = 0.1;

    ros::NodeHandle & nh;

    bool odom_init_drone1 = false;

    bool odom_init_drone2 = false;

    nav_msgs::Odometry latest_odom_drone1;

    nav_msgs::Odometry latest_odom_drone2;

    quadrotor_msgs::PositionCommand latest_cmd_drone1;

    quadrotor_msgs::PositionCommand latest_cmd_drone2;

    ros::Subscriber sub_odom_drone1;

    ros::Subscriber sub_odom_drone2;

    ros::Publisher pub_cmd_drone1;

    ros::Publisher pub_cmd_drone2;

    ros::Publisher pub_abort;

    ros::Subscriber sub_joy;

    sensor_msgs::Joy latest_joy;

    bool is_active = false;

    ros::Timer timer;

    explicit joystick_commander_t( ros::NodeHandle & _nh ): nh(_nh) {

      pub_abort = nh.advertise<std_msgs::Header>("/abort",100);

      pub_cmd_drone1 = nh.advertise<quadrotor_msgs::PositionCommand>("/drone1/position_cmd",100);

      pub_cmd_drone2 = nh.advertise<quadrotor_msgs::PositionCommand>("/drone2/position_cmd",100);

      sub_odom_drone1 = nh.subscribe<nav_msgs::Odometry>(
              "/drone1/odom",
              10,
              &joystick_commander_t::on_odom_drone1,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_odom_drone2 = nh.subscribe<nav_msgs::Odometry>(
              "/drone2/odom",
              10,
              &joystick_commander_t::on_odom_drone2,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_joy = nh.subscribe<sensor_msgs::Joy>(
              "/joy_hz",
              10,
              &joystick_commander_t::on_js,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      timer = nh.createTimer(ros::Rate(90),&joystick_commander_t::on_timer,this);

    }

    static bool is_joystick_active( sensor_msgs::Joy const & msg ){
      return msg.axes.size() > 5 &&
             ( std::abs(msg.axes[0]) > js_epsi ||
               std::abs(msg.axes[1]) > js_epsi ||
               std::abs(msg.axes[3]) > js_epsi ||
               std::abs(msg.axes[4]) > js_epsi );
    }

    static Vector3d js2vel( sensor_msgs::Joy const & joy ){
      double dir_F = -joy.axes[3];
      double dir_L = joy.axes[4];
      double dir_U = joy.axes[1];
      return Vector3d( scale_vx * dir_F, scale_vy * dir_L, scale_vz * dir_U );
    }

    void on_timer( ros::TimerEvent const & e ){

      bool joystick_active = is_joystick_active(latest_joy);

      if ( is_active && !joystick_active ) {
        on_became_inactive();
      }
      else if ( !is_active && joystick_active ) {
        on_became_active();
      }

      if ( is_active ) {
        propagate_cmd(latest_cmd_drone1);
        propagate_cmd(latest_cmd_drone2);
        latest_cmd_drone1.header.stamp = ros::Time::now();
        latest_cmd_drone2.header.stamp = ros::Time::now();
        latest_cmd_drone1.velocity = uav_utils::to_vector3_msg(js2vel(latest_joy));
        latest_cmd_drone2.velocity = uav_utils::to_vector3_msg(js2vel(latest_joy));
        pub_cmd_drone1.publish(latest_cmd_drone1);
        pub_cmd_drone2.publish(latest_cmd_drone2);
      }

    }

    static void propagate_cmd( quadrotor_msgs::PositionCommand & cmd ){
      Vector3d p = uav_utils::from_point_msg(cmd.position);
      Vector3d v = uav_utils::from_vector3_msg(cmd.velocity);
      double dt = ( ros::Time::now() - cmd.header.stamp ).toSec();
      Vector3d next_p = p + dt * v;
      cmd.position = uav_utils::to_point_msg(next_p);
    }

    void send_abort() const {
      std_msgs::Header msg;
      msg.stamp = ros::Time::now();
      pub_abort.publish(msg);
    }

    void on_became_active(){

      send_abort();

      geometry_msgs::Vector3 zero;
      zero.x = 0; zero.y = 0; zero.z = 0;

      if ( odom_init_drone1 && odom_init_drone2 ) {

        latest_cmd_drone1.header.stamp = ros::Time::now();
        latest_cmd_drone1.position = latest_odom_drone1.pose.pose.position;
        latest_cmd_drone1.velocity = zero;
        latest_cmd_drone1.acceleration = zero;
        latest_cmd_drone1.yaw_dot = 0;
        latest_cmd_drone1.yaw = uav_yaw_from_odom(latest_odom_drone1);

        latest_cmd_drone2.header.stamp = ros::Time::now();
        latest_cmd_drone2.position = latest_odom_drone2.pose.pose.position;
        latest_cmd_drone2.velocity = zero;
        latest_cmd_drone2.acceleration = zero;
        latest_cmd_drone2.yaw_dot = 0;
        latest_cmd_drone2.yaw = uav_yaw_from_odom(latest_odom_drone2);

        is_active = true;

        ROS_ERROR_STREAM("[js_cmd] activate");

      }
      else {
        ROS_ERROR_STREAM("[js_cmd] odom not initialized");
        is_active = false;
      }

    }

    void on_became_inactive(){
      is_active = false;
      ROS_ERROR_STREAM("[js_cmd] deactivate");
    }

    void on_odom_drone1( nav_msgs::OdometryConstPtr const & msg ){
      odom_init_drone1 = true;
      latest_odom_drone1 = *msg;
    }

    void on_odom_drone2( nav_msgs::OdometryConstPtr const & msg ){
      odom_init_drone2 = true;
      latest_odom_drone2 = *msg;
    }

    void on_js( sensor_msgs::JoyConstPtr const & msg ){
      latest_joy = *msg;
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"gamepad_drone_cmd_node");

  ros::NodeHandle nh("~");

  joystick_commander_t commander(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}