#include "rnw_ros/traj_uitls.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>

#include "am_traj/am_traj.hpp"
#include "am_traj/ros_msgs.h"

int main( int argc, char** argv ) {

  sleep(3);

  ros::init(argc,argv,"test_traj_node");

  ros::NodeHandle nh("~");

  ros::Publisher pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_traj_test",10,true);

  AmTraj amTrajOpt(1024,16,0.4,1,1,23,0.02);

  vector<Vector3d> wpts;
  wpts.emplace_back(2,0,1);
  wpts.emplace_back(0,0,0.5);
  wpts.emplace_back(0,0,0.5-0.05);
  wpts.emplace_back(0.5,1,0.5-0.05);
  wpts.emplace_back(0.5,-1,0.5-0.05);

  Vector3d v0 = Vector3d::Zero();

  Trajectory traj = amTrajOpt.genOptimalTrajDTC(wpts,v0,v0,v0,v0);
  ROS_INFO_STREAM("traj generated");

  auto msg = to_ros_msg(traj,ros::Time::now());
  pub_poly_traj.publish(msg);

  ros::spin();

  ros::shutdown();

  return 0;

}