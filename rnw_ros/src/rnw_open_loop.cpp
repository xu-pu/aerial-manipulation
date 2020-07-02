#include <ros/ros.h>

int main( int argc, char** argv ) {

  ros::init(argc,argv,"instance_name");

  ros::NodeHandle nh;

  ros::spin();

  return 0;

}