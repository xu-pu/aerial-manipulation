#include <iostream>
#include <deque>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <uav_utils/utils.h>

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

using Vector6d = Eigen::Matrix<double,6,1>;
using Matrix6d = Eigen::Matrix<double,6,6>;

constexpr double cov_vicon = 1e-4;

constexpr double timeout = 2;

struct item_t {

    item_t() = default;

    /**
     * Initialization with odom
     * @param msg
     */
    explicit item_t( nav_msgs::Odometry const & msg ) {
      kind = ODOM;
      odom = msg;
      q = msg.pose.pose.orientation;
      u = Vector3d::Zero();
      x.block<3,1>(0,0) = uav_utils::from_point_msg(msg.pose.pose.position);
      x.block<3,1>(3,0) = Vector3d::Zero();
      Vector6d weights; weights << cov_vicon, cov_vicon, cov_vicon, 1e3, 1e3, 1e3;
      cov = weights.asDiagonal();
      stamp = msg.header.stamp;
    }

    explicit item_t( sensor_msgs::Imu const & msg ) {
      kind = IMU;
      imu = msg;
      stamp = msg.header.stamp;
      q = msg.orientation;
      ang_vel = msg.angular_velocity;
      u = uav_utils::from_vector3_msg(msg.linear_acceleration);
    }

    enum item_type_e {
        IMU,
        ODOM
    } kind;

    sensor_msgs::Imu imu;

    nav_msgs::Odometry odom;

public: // all types should have these properties

    Vector6d x;

    Matrix6d cov;

    Vector3d u;

    Vector3d z;

    ros::Time stamp;

    geometry_msgs::Quaternion q;

    geometry_msgs::Vector3 ang_vel;

public: // utilities

    nav_msgs::Odometry to_ros_odom(){
      nav_msgs::Odometry msg;
      msg.header.frame_id = "world";
      msg.header.stamp = stamp;
      msg.pose.pose.position = uav_utils::to_point_msg(x.block<3,1>(0,0));
      msg.twist.twist.linear = uav_utils::to_vector3_msg(x.block<3,1>(3,0));
      msg.pose.pose.orientation = q;
      msg.twist.twist.angular = ang_vel;
      return msg;
    }

};

void propagate_imu(item_t & pre, item_t & cur ){

  double dt = (cur.stamp - pre.stamp).toSec();

  ROS_ASSERT_MSG(dt > 0, "time has to pass for propagate_imu");

  Matrix6d At;
  At <<   Matrix3d::Identity(), dt * Matrix3d::Identity(),
          Matrix3d::Zero()    , Matrix3d::Identity();

  Eigen::Matrix<double,6,3> Bt;
  Bt << Matrix3d::Zero(), dt * Matrix3d::Identity();

  Vector6d vQt;
  Matrix6d Qt = vQt.asDiagonal();

  cur.x = At * pre.x + Bt * cur.u;
  cur.cov = At * pre.cov * At.transpose() + Qt;

}

void propagate_odom(item_t & pre, item_t & cur ){

  propagate_imu(pre, cur);

  Eigen::Matrix<double,3,6> Ct; Ct << Matrix3d::Identity(), Matrix3d::Zero();
  Eigen::Matrix3d Rt = cov_vicon * Matrix3d::Identity();

  Eigen::Matrix<double,6,3> Kt = cur.cov * Ct.transpose() * ( Ct * cur.cov * Ct.transpose() + Rt ).inverse();
  Vector6d new_x = cur.x + Kt * ( cur.z - Ct * cur.x );
  Matrix6d new_cov = cur.cov - Kt * Ct * cur.cov;

  cur.x = new_x;
  cur.cov = new_cov;

}

struct kalman_filter_t {

    deque<item_t> timeline;

    bool propagate( sensor_msgs::Imu const & msg ){

      if (timeline.empty()) { return false; }

      item_t & pre = timeline.back();

      if ( msg.header.stamp > pre.stamp ) {
        timeline.emplace_back(msg);
        item_t & cur = timeline.back();
        propagate_imu(pre, cur);

        while ( (ros::Time::now() - timeline.front().stamp).toSec() > timeout ) {
          // make sure the queue only store messages within limited time
          timeline.pop_front();
        }

        return true;

      }
      else  {
        ROS_WARN_STREAM("[efk_odom] out of order imu");
        return false;
      }

    }

    bool propagate( nav_msgs::Odometry const & msg ){

      if (timeline.empty()) {
        timeline.emplace_back(msg);
        return true;
      }

      item_t pre;

      while ( !timeline.empty() ) {
        pre = timeline.front();
        if ( msg.header.stamp > pre.stamp ) {
          timeline.pop_front();
        }
        else {
          break;
        }
      }

      timeline.emplace_front(msg);
      item_t & cur = timeline.front();
      propagate_odom(pre, cur);

      int latency_ms = static_cast<int>((ros::Time::now() - cur.stamp).toSec() * 1000.);

      ROS_INFO("[ekf_odom] odom latency=%dms, propagate %lu imu entries", latency_ms, timeline.size());

      for ( size_t i=1; i< timeline.size(); i++ ) {
        propagate_imu(timeline.at(i - 1), timeline.at(i));
      }

      return true;

    }

    nav_msgs::Odometry odom(){
      return timeline.back().to_ros_odom();
    }

};

struct ekf_odom_node_t {

    kalman_filter_t kf;

    ros::Subscriber sub_imu;

    ros::Subscriber sub_odom;

    ros::Publisher pub_odom;

    ekf_odom_node_t(){

      ros::NodeHandle nh("~");

      sub_imu = nh.subscribe<sensor_msgs::Imu>(
              "imu",
              10,
              &ekf_odom_node_t::on_imu,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_odom = nh.subscribe<nav_msgs::Odometry>(
              "odom",
              10,
              &ekf_odom_node_t::on_odom,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      pub_odom = nh.advertise<geometry_msgs::Vector3Stamped>("odom_out",100);

    }

    void on_odom( OdometryConstPtr const & odom ){
      if ( kf.propagate(*odom) ) {
        pub_odom.publish(kf.odom());
      }
    }

    void on_imu( sensor_msgs::ImuConstPtr const & imu ){
      if ( kf.propagate(*imu) ){
        pub_odom.publish(kf.odom());
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"ekf_odom_node");

  ekf_odom_node_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}