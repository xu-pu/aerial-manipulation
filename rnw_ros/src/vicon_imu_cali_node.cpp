//
// Created by sheep on 2020/7/17.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "rnw_ros/pose_utils.h"

using sensor_msgs::ImuConstPtr;
using nav_msgs::OdometryConstPtr;

inline double deg2rad( double deg ){
  return M_PI/180.*deg;
}

struct cali_sampler_t {

    vector<Vector3d> samples_rpy;
    vector<sensor_msgs::Imu> samples_imu;
    vector<nav_msgs::Odometry> samples_odom;

    static constexpr size_t sample_threshold = 200;

    bool check_rpy( Vector3d const & rpy ){

      for ( auto const & pt : samples_rpy ) {

        double dist = (rpy-pt).norm();

        if ( dist < deg2rad(5) ) {
          return false;
        }

      }

      return true;

    }

    void sample( ImuConstPtr const & imu, OdometryConstPtr const & odom ){

      Vector3d rpy = imu2rpy(imu);

      if ( check_rpy(rpy) ) {
        samples_rpy.push_back(rpy);
        samples_imu.push_back(*imu);
        samples_odom.push_back(*odom);
        ROS_INFO_STREAM("Samples Collected: " << samples_rpy.size() << "/" << sample_threshold);
      }

    }

    bool enough_samples() const {
      return samples_rpy.size() > sample_threshold;
    }

};

cali_sampler_t cali_sampler;

void sync_callback( sensor_msgs::ImuConstPtr const & imu, nav_msgs::OdometryConstPtr const & odom ){

  if ( !cali_sampler.enough_samples() ) {
    cali_sampler.sample(imu,odom);
  }

  if ( cali_sampler.enough_samples() ) {
    ROS_INFO_STREAM("DO CERES CALIBRATION");
  }

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"vicon_imu_calib_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, "/djiros/imu", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, "/uwb_vicon_odom", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,nav_msgs::Odometry> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10),sub_imu,sub_odom);
  sync.registerCallback(sync_callback);

  ros::spin();

  return 0;

}