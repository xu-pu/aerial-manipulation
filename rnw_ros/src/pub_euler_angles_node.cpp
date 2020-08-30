#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <uav_utils/utils.h>

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

ros::Publisher pub_rpy_imu;
ros::Publisher pub_rpy_odom;

static constexpr double RAD2DEG = 180/M_PI;

void on_odom( OdometryConstPtr const & odom ){
  Eigen::Matrix3d R_FLU2FRD;
  R_FLU2FRD << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  Eigen::Matrix3d R_ENU2NED;
  R_ENU2NED << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  Vector3d rpy = odom2rpy(odom) * RAD2DEG;
  pub_rpy_odom.publish(uav_utils::to_vector3_msg(rpy));
}

void on_imu( sensor_msgs::ImuConstPtr const & imu ){
  Vector3d rpy = imu2rpy(imu) * RAD2DEG;
  pub_rpy_imu.publish(uav_utils::to_vector3_msg(rpy));
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_euler_angles_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(
          "imu",
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

  pub_rpy_imu = nh.advertise<geometry_msgs::Vector3>("/rpy/imu",10);
  pub_rpy_odom = nh.advertise<geometry_msgs::Vector3>("/rpy/odom",10);

  ros::spin();

  ros::shutdown();

  return 0;

}