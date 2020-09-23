#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

ros::Publisher pub_rpy_imu;
ros::Publisher pub_rpy_ctrl;

void on_joy_ctrl(sensor_msgs::JoyConstPtr const & joy ){
  double roll = joy->axes.at(0);
  double pitch = joy->axes.at(1);
  double thrust = joy->axes.at(2);
  double yaw = joy->axes.at(3);
  Vector3d rpy( roll*deg2rad, pitch*deg2rad, yaw*deg2rad );
  pub_rpy_ctrl.publish(eigen2rosv(rpy,joy->header));
}

void on_imu( sensor_msgs::ImuConstPtr const & imu ){
  Vector3d rpy = imu2rpy(imu);
  pub_rpy_imu.publish(eigen2rosv(rpy,imu->header));
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"dbg_atti_loop_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu",100000,on_imu);
  ros::Subscriber sub_ctrl = nh.subscribe<sensor_msgs::Joy>("joy_ctrl", 100000, on_joy_ctrl);

  pub_rpy_imu = nh.advertise<geometry_msgs::Vector3Stamped>("/dbg/atti/imu",100000);
  pub_rpy_ctrl = nh.advertise<geometry_msgs::Vector3Stamped>("/dbg/atti/ctrl",100000);

  ros::spin();

  ros::shutdown();

  return 0;

}