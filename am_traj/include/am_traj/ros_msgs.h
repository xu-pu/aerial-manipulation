//
// Created by sheep on 2020/8/12.
//

#ifndef SRC_ROS_MSGS_H
#define SRC_ROS_MSGS_H

#include "am_traj/am_traj.hpp"

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

inline quadrotor_msgs::PolynomialTrajectory to_ros_msg(Trajectory const & traj, double yaw_start, double yaw_final, ros::Time const & iniStamp ){
  quadrotor_msgs::PolynomialTrajectory msg;
  msg.header.stamp = iniStamp;
  static uint32_t traj_id = 0;
  traj_id++;
  msg.trajectory_id = traj_id;
  msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
  msg.num_order = traj[0].getOrder();
  msg.num_segment = traj.getPieceNum();
  Eigen::Vector3d initialVel, finalVel;
  initialVel = traj.getVel(0.0);
  finalVel = traj.getVel(traj.getTotalDuration());
  msg.start_yaw = yaw_start;
  msg.final_yaw = yaw_final;

  for (size_t p = 0; p < (size_t)traj.getPieceNum(); p++) {
    msg.time.push_back(traj[p].getDuration());
    msg.order.push_back(traj[p].getCoeffMat().cols() - 1);

    Eigen::VectorXd linearTr(2);
    linearTr << 0.0, msg.time[p];
    std::vector<Eigen::VectorXd> linearTrCoeffs;
    linearTrCoeffs.emplace_back(1);
    linearTrCoeffs[0] << 1;
    for (size_t k = 0; k < msg.order[p]; k++) {
      linearTrCoeffs.push_back(RootFinder::polyConv(linearTrCoeffs[k], linearTr));
    }

    Eigen::MatrixXd coefMat(3, traj[p].getCoeffMat().cols());
    for (int i = 0; i < coefMat.cols(); i++) {
      coefMat.col(i) = traj[p].getCoeffMat().col(coefMat.cols() - i - 1).head<3>();
    }
    coefMat.col(0) = coefMat.col(0).eval();

    for (int i = 0; i < coefMat.cols(); i++) {
      double coefx(0.0), coefy(0.0), coefz(0.0);
      for (int j = i; j < coefMat.cols(); j++) {
        coefx += coefMat(0, j) * linearTrCoeffs[j](i);
        coefy += coefMat(1, j) * linearTrCoeffs[j](i);
        coefz += coefMat(2, j) * linearTrCoeffs[j](i);
      }
      msg.coef_x.push_back(coefx);
      msg.coef_y.push_back(coefy);
      msg.coef_z.push_back(coefz);
    }
  }

  msg.mag_coeff = 1.0;
  msg.debug_info = "";

  return msg;

}

/**
 * Convert to ros msg while keep current yaw
 * @param traj
 * @param odom - current odom, used for yaw calc
 * @param iniStamp
 * @return
 */
inline quadrotor_msgs::PolynomialTrajectory to_ros_msg(Trajectory const & traj, nav_msgs::Odometry const & odom ,ros::Time const & iniStamp ){
  double cur_yaw = uav_utils::get_yaw_from_quaternion(uav_utils::from_quaternion_msg(odom.pose.pose.orientation));
  return to_ros_msg(traj, cur_yaw, cur_yaw, iniStamp);
}

#endif //SRC_ROS_MSGS_H
