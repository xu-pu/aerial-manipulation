//
// Created by sheep on 2020/7/5.
//
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

#include "poly_traj/polynomial_traj.h"

using namespace std;

using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

PolynomialTraj traj;

ros::Publisher pub_path_setpoint;

MatrixXd toXd( vector<Vector3d> const & src ){
  MatrixXd dst(src.size(),3);
  for ( size_t i=0; i<src.size(); i++ ) {
    dst.block<1,3>(i,0) = src.at(i).transpose();
  }
  return dst;
}

VectorXd toXd( vector<double> const & src ){
  VectorXd dst(src.size());
  for ( size_t i=0; i<src.size(); i++ ){
    dst(i) = src.at(i);
  }
  return dst;
}

VectorXd gen_time_intervals( double dt, vector<Vector3d> const & waypoints ){
  size_t N = waypoints.size() - 1;
  VectorXd dst(N);
  for ( size_t i=0; i<N; i++ ){
    dst(i) = dt;
  }
  return dst;
}

geometry_msgs::Point eigen2ros( Vector3d const & src ){
  geometry_msgs::Point dst;
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  return dst;
}

void pub_setpoint(){

  static ros::Time init_time;
  static bool init = false;

  static nav_msgs::Path path;
  path.header.frame_id = "world";


  if (!init) {
    init_time = ros::Time::now();
    init = true;
  }

  auto cur_time = ros::Time::now();
  double dt = ( cur_time - init_time ).toSec();

  if ( dt < traj.getTimeSum() ) {

    Vector3d x = traj.evaluate(dt);
    Vector3d x_dot = traj.evaluateVel(dt);
    Vector3d x_dot_dot = traj.evaluateAcc(dt);

    ROS_INFO_STREAM(dt);
    ROS_INFO_STREAM(x);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cur_time;
    pose.pose.position = eigen2ros(x);

    path.poses.push_back(pose);

    pub_path_setpoint.publish(path);

  }
  else {
    ROS_INFO_STREAM("Traj Complete");
  }

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_frames_node");

  ros::NodeHandle nh("~");

  pub_path_setpoint = nh.advertise<nav_msgs::Path>("/traj/setpoint",10);

  vector<Vector3d> waypoints = {
          Vector3d {0,0,0},
          Vector3d {0.5,0.1,0},
          Vector3d {1,-0.1,0},
          Vector3d {1.5,0.1,0}
  };

  MatrixXd POS = toXd(waypoints);
  VectorXd TIMES = gen_time_intervals(1,waypoints);
  PolynomialTraj test_traj = minSnapTraj(POS,Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),TIMES);

  traj = test_traj;

  ros::Rate rate(100);

  while (ros::ok()) {

    pub_setpoint();

    ros::spinOnce();

    rate.sleep();

  }

  ros::shutdown();

  return 0;

}