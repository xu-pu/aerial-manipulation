#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <uav_utils/converters.h>

using namespace uav_utils;

ros::Publisher pub_imu_fixed;

bool acc_ready = false;

Eigen::Vector3d acc_w;

nav_msgs::Odometry last_odom;

void on_imu( sensor_msgs::ImuConstPtr const & msg ){
  if ( acc_ready ) {
    sensor_msgs::Imu fix = *msg;
    auto acc_b =
    fix.linear_acceleration = to_vector3_msg(
            from_quaternion_msg(msg->orientation).inverse()*(acc_w+Eigen::Vector3d(0,0,9.81))
    );
    pub_imu_fixed.publish(fix);
  }
}

void on_odom( nav_msgs::OdometryConstPtr const & msg ){
  double dt = (msg->header.stamp - last_odom.header.stamp).toSec();
  acc_ready = dt < 0.5;
  if ( acc_ready ) {
    acc_w = (from_vector3_msg(msg->twist.twist.linear)-from_vector3_msg(last_odom.twist.twist.linear)) / dt;
  }
  last_odom = *msg;
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"sim_imu_acc_node");

  ros::NodeHandle nh("~");

  pub_imu_fixed = nh.advertise<sensor_msgs::Imu>("imu_out",1000);

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(
          "imu_in",
          10,
          on_imu,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "odom",
          10,
          on_odom,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}

