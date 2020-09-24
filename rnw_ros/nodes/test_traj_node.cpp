#include "rnw_ros/traj_uitls.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>

#include "am_traj/am_traj.hpp"
#include "am_traj/ros_msgs.h"

#include "rnw_ros/rnw_utils.h"

int main( int argc, char** argv ) {

  ros::init(argc,argv,"test_traj_node");

  ros::NodeHandle nh("~");

  rnw_config_t rnw_config;
  rnw_config.load_from_ros(nh);

  ros::Publisher pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_traj_test",10,true);

  AmTraj amTrajOpt(1024,16,0.4,1,1,23,0.02);

  Matrix3d R_tip = Matrix3d::Identity();
  Vector3d T_tip = {0,0,1};

  Vector3d cur_pos = {2,3,1};

  vector<Vector3d> wpts = gen_wpts_insert_topple(rnw_config);

  vector<Vector3d> waypoints = transform_pts(wpts,R_tip,T_tip);

  waypoints.insert(waypoints.begin(),cur_pos);

  Vector3d v0 = Vector3d::Zero();

  Trajectory traj = amTrajOpt.genOptimalTrajDTC(waypoints,v0,v0,v0,v0);
  ROS_INFO_STREAM("traj generated");

  auto msg = to_ros_msg(traj,0,0,ros::Time::now());

  ros::Rate rate(1);

  while(ros::ok()){
    pub_poly_traj.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();

  return 0;

}