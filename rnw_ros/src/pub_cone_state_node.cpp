#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <quadrotor_msgs/Float64Stamped.h>

#include "uav_utils/geometry_utils.h"

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"
#include "rnw_ros/ConeState.h"

Vector3d cone_rot2euler( Matrix3d const & R ){
  double psi=0,theta=0,phi=0;

  auto r00 = R(0,0);
  auto r02 = R(0,2);
  auto r10 = R(1,0);
  auto r12 = R(1,2);
  auto r20 = R(2,0);
  auto r21 = R(2,1);
  auto r22 = R(2,2);

  if ( r22 == -1 ) {
    theta = M_PI/2;
    phi = 0;
    psi = atan2(r00,-r10);

  }
  else if ( r22 == 1 ){
    theta = 0;
    phi = 0;
    psi = atan2(-r00,r10);
  }
  else {
    theta = atan2(sqrt(r02*r02+r12*r12),r22);
    phi = atan2(r21,-r20);
    psi = atan2(-r02,r12);
  }

  return { psi, theta, phi };

}

struct cone_state_estimator_t {

    ros::Publisher pub_cone_state;

    ros::Publisher pub_odom_dt;

    double odom_frequency = 50;

    double odom_timeout = 1;

    nav_msgs::Odometry latest_odom;

    bool init = false;

    bool cut_euler_velocity = false;

    double max_euler_velocity = numeric_limits<double>::max();

    explicit cone_state_estimator_t( ros::NodeHandle & nh ){
      pub_cone_state = nh.advertise<rnw_ros::ConeState>("state",10);
      pub_odom_dt = nh.advertise<quadrotor_msgs::Float64Stamped>("dt",10);
      cut_euler_velocity = get_param_default(nh,"cut_euler_velocity",false);
      max_euler_velocity = get_param_default(nh,"max_euler_velocity",numeric_limits<double>::max());
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg ){

      if ( !init ) {
        latest_odom = *msg;
        init = true;
        ROS_INFO_STREAM("[Cone] Initialized");
        return;
      }
      else if ( msg_time_diff(latest_odom,*msg) > odom_timeout ) {
        latest_odom = *msg;
        ROS_ERROR_STREAM("[Cone] Odom Timeout, re-initialized!");
        return;
      }
      else if ( msg_time_diff(latest_odom,*msg) < 0 ) {
        latest_odom = *msg;
        ROS_ERROR_STREAM("[Cone] Message out of order, re-initialized!");
        return;
      }

      auto pre = latest_odom;
      auto cur = *msg;
      latest_odom = *msg;

      double dt = msg_time_diff(pre,cur);

      quadrotor_msgs::Float64Stamped msg_dt;
      msg_dt.header.stamp = msg->header.stamp;
      msg_dt.value = dt;
      pub_odom_dt.publish(msg_dt);

      Vector3d euler_cur = cone_rot2euler(odom2R(cur));
      Vector3d euler_pre = cone_rot2euler(odom2R(pre));

      Vector3d euler_diff = euler_cur - euler_pre;
      //ROS_INFO_STREAM("before " << euler_diff.transpose());
      euler_diff(0) = uav_utils::normalize_angle(euler_diff(0));
      euler_diff(1) = uav_utils::normalize_angle(euler_diff(1));
      euler_diff(2) = uav_utils::normalize_angle(euler_diff(2));
      //ROS_INFO_STREAM("after: " << euler_diff.transpose());

      Vector3d euler_vel = euler_diff/dt;

      if ( cut_euler_velocity ) {
        euler_vel(0) = min(euler_vel(0),max_euler_velocity);
        euler_vel(1) = min(euler_vel(1),max_euler_velocity);
        euler_vel(2) = min(euler_vel(2),max_euler_velocity);
        euler_vel(0) = max(euler_vel(0),-max_euler_velocity);
        euler_vel(1) = max(euler_vel(1),-max_euler_velocity);
        euler_vel(2) = max(euler_vel(2),-max_euler_velocity);
      }

      rnw_ros::ConeState msg_cone;
      msg_cone.header.stamp = msg->header.stamp;
      msg_cone.odom = cur;
      msg_cone.euler_angles = eigen2rosv(euler_cur);
      msg_cone.euler_angles_velocity = eigen2rosv(euler_vel);
      pub_cone_state.publish(msg_cone);

    }

};


int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_state_node");

  ros::NodeHandle nh("~");

  cone_state_estimator_t cone_state_estimator(nh);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("odom",10,&cone_state_estimator_t::on_odom,&cone_state_estimator);

  ros::spin();

  ros::shutdown();

  return 0;

}