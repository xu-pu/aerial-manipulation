#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"
#include "rnw_msgs/ConeState.h"

ros::Publisher pub_est_radius;
ros::Publisher pub_true_radius;
ros::Publisher pub_cone_yaw;
ros::Publisher pub_cp_height;
ros::Publisher pub_nutation_deg;
ros::Publisher pub_precession_plus;

rnw_config_t rnw_config;

void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg ){

  Vector3d D = uav_utils::from_point_msg(msg->disc_center);
  Vector3d G = uav_utils::from_point_msg(msg->contact_point);
  double r = (D-G).norm();

  quadrotor_msgs::Float64Stamped msg2;
  msg2.header.stamp = msg->header.stamp;
  msg2.value = r;
  pub_est_radius.publish(msg2);

  msg2.value = rnw_config.cone.radius;
  pub_true_radius.publish(msg2);

  msg2.value = rad2deg * cone_yaw(*msg);
  pub_cone_yaw.publish(msg2);

  msg2.value = rad2deg * uav_utils::normalize_angle(msg->euler_angles.x + M_PI_2);
  pub_precession_plus.publish(msg2);

  msg2.value = msg->tip.z - rnw_config.ground_z;
  pub_cp_height.publish(msg2);

  msg2.value = msg->euler_angles.y * rad2deg;
  pub_nutation_deg.publish(msg2);

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_cone_debug_info_node");

  ros::NodeHandle nh("~");

  rnw_config.load_from_ros(nh);

  ros::Subscriber sub_odom = nh.subscribe<rnw_msgs::ConeState>(
          "/cone/state",
          10,
          on_cone_state,
          nullptr,
          ros::TransportHints().tcpNoDelay()
  );

  pub_est_radius = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/radius_est",100);
  pub_true_radius = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/radius_true",100);
  pub_cone_yaw = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/yaw",100);
  pub_nutation_deg = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/nutation_deg",100);
  pub_cp_height = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/control_point_height", 100);
  pub_precession_plus = nh.advertise<quadrotor_msgs::Float64Stamped>("/cone/precession_plus", 100);

  ros::spin();

  ros::shutdown();

  return 0;

}