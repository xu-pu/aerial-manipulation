#include <ros/ros.h>

#include "rnw_ros/rnw_utils.h"

#include <rnw_msgs/WalkingState.h>
#include <quadrotor_msgs/Float64Stamped.h>

ros::Publisher pub_energy;
ros::Publisher pub_energy_T;
ros::Publisher pub_energy_V;
ros::Publisher pub_energy_diff;

rnw_config_t rnw_config;

Vector3d pt_on_cone( Vector3d const & pt, rnw_msgs::ConeState const & cone_state ){
  Quaterniond R = uav_utils::from_quaternion_msg(cone_state.odom.pose.pose.orientation);
  Vector3d T = uav_utils::from_point_msg(cone_state.odom.pose.pose.position);
  return R * pt + T;
}

void calc_energy_diff( rnw_msgs::WalkingStateConstPtr const & msg ){
  static bool init = false;
  static rnw_msgs::WalkingState pre_msg;
  static size_t cur_step;
  static Vector3d CoM_last_step;
  static double latest_ed = 0;

  double m = 0.5;
  double g = 9.8;

  Vector3d cur_CoM = pt_on_cone(rnw_config.cone.CoM(),msg->cone_state);

  if ( init ) {
    if ( msg->step_count > cur_step ) {
      cur_step = msg->step_count;
      latest_ed = m * g * ( cur_CoM.z() - CoM_last_step.z() );
      CoM_last_step = cur_CoM;
    }
  }
  else {
    init = true;
    cur_step = msg->step_count;
    CoM_last_step = cur_CoM;
  }

  quadrotor_msgs::Float64Stamped msg_out;
  msg_out.header = msg->header;
  msg_out.value = latest_ed;
  pub_energy_diff.publish(msg_out);

}

void on_walking_state( rnw_msgs::WalkingStateConstPtr const & msg ){

  calc_energy_diff(msg);

  double com_z = 0.05;
  double com_x = 0.08;

  Vector3d CoM = rnw_config.cone.base_center;
  CoM.z() = CoM.z() + com_z;
  CoM.x() = CoM.x() + com_x;

  static rnw_msgs::WalkingState pre_msg;
  static bool init = false;

  if ( !init ) {
    pre_msg = *msg;
    init = true;
    return;
  }

  rnw_msgs::WalkingState cur_msg = *msg;

  double dt = ( cur_msg.header.stamp - pre_msg.header.stamp ).toSec(); // sec

  Vector3d pt_pre = pt_on_cone(CoM,pre_msg.cone_state);
  Vector3d pt_cur = pt_on_cone(CoM,cur_msg.cone_state);

  double dist = ( pt_cur - pt_pre ).norm(); // m
  double v = dist/dt; // m/s

  double m = 0.5; // kg
  double g = 9.8;
  double h = pt_cur.z(); // m

  double T = 0.5 * m * v * v;
  double V = m * g * h;

  double E = T+V;

  quadrotor_msgs::Float64Stamped msg_out;
  msg_out.header = msg->header;

  msg_out.value = E;
  pub_energy.publish(msg_out);

  msg_out.value = T;
  pub_energy_T.publish(msg_out);

  msg_out.value = V;
  pub_energy_V.publish(msg_out);

  pre_msg = cur_msg;

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_energy_node");

  ros::NodeHandle nh("~");

  rnw_config.load_from_ros(nh);

  ros::Subscriber sub_imu = nh.subscribe<rnw_msgs::WalkingState>(
          "/rnw/walking_state/session_1",
          100,
          on_walking_state,
          ros::TransportHints().tcpNoDelay()
  );

  pub_energy = nh.advertise<quadrotor_msgs::Float64Stamped>("/rnw/cone_energy",100);

  pub_energy_T = nh.advertise<quadrotor_msgs::Float64Stamped>("/rnw/cone_energy_T",100);

  pub_energy_V = nh.advertise<quadrotor_msgs::Float64Stamped>("/rnw/cone_energy_V",100);

  pub_energy_diff = nh.advertise<quadrotor_msgs::Float64Stamped>("/rnw/cone_energy_diff",100);

  ros::spin();

  ros::shutdown();

  return 0;

}