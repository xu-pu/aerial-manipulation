//
// Created by sheep on 2020/7/5.
//

#ifndef SRC_TRAJ_UITLS_H
#define SRC_TRAJ_UITLS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Dense>

#include "quadrotor_msgs/PositionCommand.h"
#include "poly_traj/polynomial_traj.h"

using namespace std;

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

VectorXd gen_time_intervals( vector<double> const & intervals, vector<Vector3d> const & waypoints ){
  assert( waypoints.size()-1 == intervals.size() );
  size_t N = intervals.size();
  VectorXd dst(N);
  for ( size_t i=0; i<N; i++ ){
    dst(i) = intervals.at(i);
  }
  return dst;
}

geometry_msgs::Vector3Stamped eigen2rosv( Vector3d const & src, std_msgs::Header const & header ){
  geometry_msgs::Vector3Stamped dst;
  dst.header = header;
  dst.vector.x = src.x();
  dst.vector.y = src.y();
  dst.vector.z = src.z();
  return dst;
}

vector<Vector3d> gen_waypoint_zigzag( size_t cycles, double step_x, double step_y ){

  vector<Vector3d> dst;
  dst.emplace_back(0,0,0);

  for ( size_t i=0; i<cycles; i++ ) {
    dst.emplace_back((2*i+1)*step_x,step_y,0);
    dst.emplace_back((2*i+2)*step_x,-step_y,0);
  }

  dst.emplace_back((2*cycles+1)*step_x,0,0);

  return dst;

}

vector<Vector3d> transform_pts( vector<Vector3d> const & src, Matrix3d const & R, Vector3d const & T ){
  vector<Vector3d> dst;
  for ( auto const & pt : src ) {
    dst.emplace_back(R*pt+T);
  }
  return dst;
}

#endif //SRC_TRAJ_UITLS_H
