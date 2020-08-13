#include "rnw_ros/traj_uitls.h"

#include <quadrotor_msgs/PolynomialTrajectory.h>

#include "am_traj/am_traj.hpp"
#include "am_traj/ros_msgs.h"

int main( int argc, char** argv ) {

  sleep(3);

  ros::init(argc,argv,"rnw_controller_node");

  ros::NodeHandle nh("~");

  ros::Publisher pub_poly_traj = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_traj_test",10,true);

  ros::spin();

  ros::shutdown();

  return 0;

}