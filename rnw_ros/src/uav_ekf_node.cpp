#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>

#include "rnw_ros/pose_utils.h"

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
ros::Publisher path_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
Matrix<double,6,6> Rt = Matrix<double,6,6>::Identity();

using Matrix15 = Matrix<double,15,15>;
using Vector15 = Matrix<double,15,1>;
using Vector6 = Matrix<double,6,1>;
using Vector6d = Matrix<double,6,1>;
using Matrix6d = Matrix<double,6,6>;

Vector3d rot2euler( Matrix3d const & R ){
//  double phi = asin(R(1,2));
//  double psi = atan2(-R(1,0)/cos(phi),R(1,1)/cos(phi));
//  double theta = atan2(-R(0,2)/cos(phi),R(2,2)/cos(phi));
//  return { phi,theta,psi };
  return quat2eulers(Quaterniond(R));
}

Matrix3d euler_linear( Vector3d const & euler ){

  double phi = euler(0);
  double theta = euler(1);

  double sp = sin(phi);
  double cp = cos(phi);
  double st = sin(theta);
  double ct = cos(theta);

  Matrix3d rst = Matrix3d::Zero();

  rst(0,0) = ct;
  rst(0,2) = -cp*st;

  rst(1,1) = 1;
  rst(1,2) = sp;

  rst(2,0) = st;
  rst(2,2) = cp*ct;

  return rst;

}

double sec( double r ) {
  return 1/cos(r);
}

Matrix3d Rx( double phi ){
  Matrix3d R;
  R <<
    1, 0, 0,
          0, cos(phi), -sin(phi),
          0, sin(phi), cos(phi);
  return R;
}

Matrix3d Ry( double theta ){
  Matrix3d R;
  R <<
    cos(theta), 0, sin(theta),
          0, 1, 0,
          -sin(theta), 0, cos(theta);
  return R;
}

Matrix3d Rz( double psi ){
  Matrix3d R;
  R <<
    cos(psi), -sin(psi), 0,
          sin(psi), cos(psi), 0,
          0, 0, 1;
  return R;
}

Matrix3d euler2rot( Vector3d const & euler ){

  double roll = euler(0);
  double pitch = euler(1);
  double yaw = euler(2);
//
//  double sr = sin(roll);
//  double cr = cos(roll);
//  double sp = sin(pitch);
//  double cp = cos(pitch);
//  double sy = sin(yaw);
//  double cy = cos(yaw);
//
//  Matrix3d rst;
//  rst(0,0) = cy*cp - sr*sy*sp;
//  rst(0,1) = -cr*sy;
//  rst(0,2) = cy*sp+cp*sr*sy;
//
//  rst(1,0) = cp*sy+cy*sr*sp;
//  rst(1,1) = cr*cy;
//  rst(1,2) = sy*sp-cy*cp*sr;
//
//  rst(2,0) = -cr*sp;
//  rst(2,1) = sr;
//  rst(2,2) = cr*cp;

  return Rz(yaw)*Ry(pitch)*Rx(roll);

}

Matrix3d Rx_dot( double phi ){
  Matrix3d R;
  R <<
    0, 0, 0,
          0, -sin(phi), -cos(phi),
          0, cos(phi), -sin(phi);
  return R;
}

Matrix3d Ry_dot( double theta ){
  Matrix3d R;
  R <<
    -sin(theta), 0, cos(theta),
          0, 0, 0,
          -cos(theta), 0, -sin(theta);
  return R;
}

Matrix3d Rz_dot( double psi ){
  Matrix3d R;
  R <<
    -sin(psi), -cos(psi), 0,
          cos(psi), -sin(psi), 0,
          0, 0, 0;
  return R;
}

Matrix3d G_dot_phi(double phi, double theta){
  Matrix3d rst;
  rst << 0, 0, 0,
          sin(theta)*sec(phi)*sec(phi), 0, -cos(theta)*sec(phi)*sec(phi),
          -sin(theta)*sec(phi)*tan(phi), 0, cos(theta)*sec(phi)*tan(phi);
  return rst;
}

Matrix3d G_dot_theta(double phi, double theta){
  Matrix3d rst;
  rst << -sin(theta), 0, cos(theta),
          cos(theta)*tan(phi), 0, sin(theta)*tan(phi),
          -cos(theta)*sec(phi), 0, -sin(theta)*sec(phi);
  return rst;
}

/**
 *
 * @param s - state vector
 * @param u - command vector
 * @return At linearization
 */
Matrix15 calc_At( Vector15 const & s, Vector6 const & u ){

  Matrix15 At = Matrix15::Zero();

  Vector3d x2 = s.segment(3,3);
  double phi = x2(0);
  double theta = x2(1);
  double psi = x2(2);
  Matrix3d R = euler2rot(x2);

  // derivative of x1
  //======================================

  At.block<3,3>(0,6) = Matrix3d::Identity();


  // derivative of x2
  //======================================

  Vector3d v1 = u.segment(0,3) - s.segment(9,3);

  At.block<3,1>(3,3) = G_dot_phi(phi,theta)*v1;
  At.block<3,1>(3,4) = G_dot_theta(phi,theta)*v1;
  At.block<3,1>(3,5).setZero();

  At.block<3,3>(3,9) = -euler_linear(x2).inverse();

  // derivative of x3
  //======================================

  Vector3d acc = u.segment(3,3);
  Vector3d x5 = s.segment(12,3);
  Vector3d v2 = acc-x5;

  At.block<3,1>(6,3) = Rz(psi)*Rx_dot(phi)*Ry(theta)*v2;
  At.block<3,1>(6,4) = Rz(psi)*Rx(phi)*Ry_dot(theta)*v2;
  At.block<3,1>(6,5) = Rz_dot(psi)*Rx(phi)*Ry(theta)*v2;

  At.block<3,3>(6,12) = -R;

  return At;

}

/**
 *
 * @param s - state vector
 * @param u - command vector
 * @return Bt linearization
 */
Matrix<double,15,6> calc_Bt( Vector15 const & s, Vector6 const & u ){

  Matrix<double,15,6> Bt = Matrix<double,15,6>::Zero();

  Vector3d x2 = s.segment(3,3);

  Bt.block<3,3>(3,0) = euler_linear(x2).inverse();
  Bt.block<3,3>(6,3) = euler2rot(x2);

  return Bt;

}


/**
 *
 * @param s - state vector
 * @param u - command vector
 * @return Ut linearization
 */
Matrix<double,15,12> calc_Ut( Vector15 const & s, Vector6 const & u ){

  Matrix<double,15,12> Ut = Matrix<double,15,12>::Zero();

  Vector3d x2 = s.segment(3,3);

  Ut.block<3,3>(3,0) = -euler_linear(x2).inverse();
  Ut.block<3,3>(6,3) = -euler2rot(x2);
  Ut.block<3,3>(9,6).setIdentity();
  Ut.block<3,3>(12,9).setIdentity();

  return Ut;

}

Vector15 calc_state_deri( Vector15 const & s, Vector6 const & u ){

  Vector3d g(0,0,0);

  Vector3d x2 = s.segment(3,3);
  Vector3d x3 = s.segment(6,3);
  Vector3d x4 = s.segment(9,3);
  Vector3d x5 = s.segment(12,3);

  Vector3d u1 = u.segment(0,3);
  Vector3d u2 = u.segment(3,3);

  Vector15 rst = Vector15::Zero();
  rst.segment(0,3) = x3;
  rst.segment(3,3) = euler_linear(x2).inverse() * (u1-x4);
  rst.segment(6,3) = g + euler2rot(x2)*(u2-x5);

  return rst;

}


Matrix<double,6,15> mat_Ct(){
  Matrix<double,6,15> Ct;
  Ct.setZero();
  Ct.block<3,3>(0,0).setIdentity();
  Ct.block<3,3>(3,3).setIdentity();
  return Ct;
}

double euler_rounding(double val){
  while ( val > M_PI ) {
    val -= 2*M_PI;
  }
  while (val < -M_PI) {
    val += 2*M_PI;
  }
  return val;
}

Vector3d euler_diff( Vector3d const & v1, Vector3d const & v2 ){
  Vector3d rst = v1-v2;
  rst(0) = euler_rounding(rst(0));
  rst(1) = euler_rounding(rst(1));
  rst(2) = euler_rounding(rst(2));
  return rst;
}

Vector6 calc_measure_err( Vector15 const & state, Matrix3d const & R, Vector3d const & T ){

  Vector3d x1 = state.segment(0,3);
  Vector3d x2 = state.segment(3,3);

  Vector3d obs1 = T;
  Vector3d obs2 = rot2euler(R);

  Vector6 dd = Vector6::Zero();
  dd.segment(0,3) = obs1 - x1;
  dd.segment(3,3) = euler_diff(obs2,x2);
  return dd;

}

void publish_path( Vector3d const & T ){
  static nav_msgs::Path path_;
  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x = T.x();
  path_pose.pose.position.y = T.y();
  path_pose.pose.position.z = T.z();
  path_.poses.push_back(path_pose);
  path_pub.publish(path_);
}

void publish_odom( Matrix3d const & R, Vector3d const & T, Vector3d const & T_dot ){
  Quaterniond quat(R);
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = T(0);
  odom.pose.pose.position.y = T(1);
  odom.pose.pose.position.z = T(2);
  odom.pose.pose.orientation.w = quat.w();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.twist.twist.linear.x = T_dot.x();
  odom.twist.twist.linear.y = T_dot.y();
  odom.twist.twist.linear.z = T_dot.z();
  odom_pub.publish(odom);
}

void publish_state( Vector15 const & s, bool ignore_position ){

  Vector3d x1 = s.segment(0,3);
  Vector3d x2 = s.segment(3,3);
  Vector3d x3 = s.segment(6,3);

  if (ignore_position) {
    x1 = Vector3d::Zero();
  }

  Matrix3d R = euler2rot(x2);

  publish_path(x1);
  publish_odom(R,x1,x3);
  publish_frame(R,x1,"ekf","world");

}

struct ekf_estimator_t {

    bool ekf_init = false;

    Vector15 ekf_state;

    Matrix15 ekf_cov;

    bool imu_init = false;

    ros::Time t_last;

    ros::Time t_boot;

    bool enable_measurement_update = true;

    bool ignore_position = false;

    Matrix3d Rbc;

    Vector3d Tbc;

    void log_state(){
      Vector3d x4 = ekf_state.segment(9,3);
      Vector3d x5 = ekf_state.segment(12,3);
      cout << "gyro_bias: " << x4.transpose() << endl;
      cout << "acc_bias: " << x5.transpose() << endl;
    }

    ekf_estimator_t(): Rbc(Quaterniond(1, 0, 0, 0)), Tbc(0,0,0) { }

    void normalize_euler_angles(){
      ekf_state(3) = euler_rounding(ekf_state(3));
      ekf_state(4) = euler_rounding(ekf_state(4));
      ekf_state(5) = euler_rounding(ekf_state(5));
    }

    void update_imu( ros::Time const & t, Vector6d const & u ){

      //your code for propagation

      if ( !imu_init || !ekf_init ) {
        t_last = t;
        t_boot = t_last;
        imu_init = true;
        return;
      }

      double dt = ( t - t_last ).toSec();
      t_last = t;

      Matrix15 Ft = Matrix15::Identity() + dt*calc_At(ekf_state,u);
      Matrix<double,15,12> Vt = dt * calc_Ut(ekf_state,u);
      ekf_cov = Ft*ekf_cov*Ft.transpose() + Vt*Q*Vt.transpose();
      ekf_cov = ekf_cov + Vt*Q*Vt.transpose();

      Vector15 x_dot = calc_state_deri(ekf_state,u);
      ekf_state = ekf_state + dt*x_dot;

      publish_state(ekf_state,ignore_position);

      normalize_euler_angles();

    }

    void update_tag( ros::Time const & t, Matrix3d const & R, Vector3d const & T ){

      if ( !ekf_init ) {
        ekf_state.segment(0,3) = T;
        ekf_state.segment(3,3) = rot2euler(R);
        ekf_state.segment(6,3).setZero();
        ekf_state.segment(9,3).setZero();
        ekf_state.segment(12,3).setZero();
        ekf_cov.setIdentity();
        ekf_init = true;
      }
      else {
        // measurement update
        Matrix<double,6,15> Ct = mat_Ct();
        Matrix<double,15,6> Kt = ekf_cov * Ct.transpose()
                                 * (Ct*ekf_cov*Ct.transpose()+Rt).inverse();
        Vector6d err = calc_measure_err(ekf_state,R,T);

        if (enable_measurement_update) {
          ekf_state = ekf_state + Kt*err;
          ekf_cov = ekf_cov - Kt*Ct*ekf_cov;
        }

      }

      normalize_euler_angles();

    }

};

ekf_estimator_t estimator;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {

  Vector6 u;
  u(0) = msg->angular_velocity.x;
  u(1) = msg->angular_velocity.y;
  u(2) = msg->angular_velocity.z;

  u(3) = msg->linear_acceleration.x;
  u(4) = msg->linear_acceleration.y;
  u(5) = msg->linear_acceleration.z;

  estimator.update_imu(msg->header.stamp,u);

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;


bool check_odom( Matrix3d const & R, Vector3d const & T ){

  static bool init = false;
  static Matrix3d pre_R;
  static Vector3d pre_T;

  if ( init ) {
    if ((T-pre_T).norm() > 1 ) {
      return false;
    }
  }
  else {
    init = true;
  }

  pre_R = R;
  pre_T = T;

  return true;

}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  //your code for update
  // camera position in the IMU frame = (0.05, 0.05, 0)
  // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
  //					   RotationMatrix << 1, 0, 0,
  //							             0, -1, 0,
  //                                       0, 0, -1;

  auto in_pos = msg->pose.pose.position;
  auto in_quat = msg->pose.pose.orientation;
  Quaterniond quat(in_quat.w,in_quat.x,in_quat.y,in_quat.z);

  Matrix3d Rwb = quat.toRotationMatrix();
  Vector3d Twb(in_pos.x,in_pos.y,in_pos.z);

  publish_frame(Rwb,Twb,"tag","world");

  Vector3d euler = rot2euler(Rwb);
  ROS_INFO_STREAM("roll: " << euler(0) << ", pitch: " << euler(1) << ", yaw: " << euler(2));

  estimator.update_tag(msg->header.stamp,Rwb,Twb);

  Matrix3d RR = euler2rot(rot2euler(Rwb));
  publish_frame(RR,Twb,"test_euler","world");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf");
  ros::NodeHandle n("~");
  ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
  ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
  odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
  path_pub = n.advertise<nav_msgs::Path>("ekf_path", 100);
  Rcam = Quaterniond(1, 0, 0, 0).toRotationMatrix();
  cout << "R_cam" << endl << Rcam << endl;
  // Q imu covariance matrix; Rt visual odomtry covariance matrix
  // You should also tune these parameters

  // angular velocity
  Q.block<3,3>(0,0) = 1 * Matrix3d::Identity();
  // acc
  Q.block<3,3>(3,3) = 1 * Matrix3d::Identity();
  // ang bias
  Q.block<3,3>(6,6) = 0.1 * Matrix3d::Identity();
  // acc bias
  Q.block<3,3>(9,9) = 0.1 * Matrix3d::Identity();

  // position obs
  Rt.topLeftCorner(3, 3) = 0.001 * Rt.topLeftCorner(3, 3);
  // rotation obs
  Rt.bottomRightCorner(3, 3) = 0.01 * Rt.bottomRightCorner(3, 3);
  //Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

  ros::spin();
}