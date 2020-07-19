//
// Created by sheep on 2020/7/17.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <ceres/ceres.h>

#include "rnw_ros/pose_utils.h"

using sensor_msgs::ImuConstPtr;
using nav_msgs::OdometryConstPtr;

inline double deg2rad( double deg ){
  return M_PI/180.*deg;
}

inline double rad2deg( double rad ){
  return 180./M_PI*rad;
}

struct cali_sampler_t {

    vector<Vector3d> samples_rpy;
    vector<sensor_msgs::Imu> samples_imu;
    vector<nav_msgs::Odometry> samples_odom;

    static constexpr size_t sample_threshold = 50;
    static constexpr double deg_threshold = 1;

    bool check_rpy( Vector3d const & rpy ){

      for ( auto const & pt : samples_rpy ) {

        double dist = (rpy-pt).norm();

        if ( dist < deg2rad(deg_threshold) ) {
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


struct ViconImuAlignError {

    Quaterniond R_VICON_MARKER;

    double imu_roll;
    double imu_pitch;

    ViconImuAlignError( sensor_msgs::Imu const & imu, nav_msgs::Odometry const & odom ){
      R_VICON_MARKER.x() = odom.pose.pose.orientation.x;
      R_VICON_MARKER.y() = odom.pose.pose.orientation.y;
      R_VICON_MARKER.z() = odom.pose.pose.orientation.z;
      R_VICON_MARKER.w() = odom.pose.pose.orientation.w;
      Vector3d imu_rpy = imu2rpy(imu);
      imu_roll = imu_rpy(0);
      imu_pitch = imu_rpy(1);
    }

    bool operator()( const double * const params, double * residual ) const {

      Eigen::Quaterniond R_MARKER_FLU(params[3],params[0],params[1],params[2]);

      Eigen::Quaterniond R_VICON_FLU = R_VICON_MARKER * R_MARKER_FLU;

      Vector3d rpy = odom2rpy(R_VICON_FLU);

      residual[0] = imu_roll - rpy.x(); // err_roll
      residual[1] = imu_pitch - rpy.y(); // err_pitch

      return true;

    }

};

void calibrate_vicon_imu( cali_sampler_t & sample ){

  using namespace ceres;

  assert( sample.enough_samples() );

  ceres::Problem problem;

  Quaterniond R_MARKER_FLU = Quaterniond::Identity();

  double * data = R_MARKER_FLU.coeffs().data();

  for ( size_t i=0; i<sample.samples_rpy.size(); i++ ) {

    auto const & imu = sample.samples_imu.at(i);
    auto const & odom = sample.samples_odom.at(i);

    ceres::CostFunction * costFunction = new NumericDiffCostFunction<ViconImuAlignError,CENTRAL,2,4>(new ViconImuAlignError(imu,odom));

    //ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
    problem.AddResidualBlock(costFunction,nullptr,data);

  }

  ceres::LocalParameterization* quaternionParameterization =
          new ceres::EigenQuaternionParameterization;

  problem.SetParameterization(data,quaternionParameterization);

  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.num_threads = 1;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  ROS_INFO_STREAM("R_MARKER_FLU: " << R_MARKER_FLU.coeffs().transpose() );
  ROS_INFO_STREAM("Magnitude: " << rad2deg(Eigen::AngleAxisd(R_MARKER_FLU).angle()) );

}


void sync_callback( sensor_msgs::ImuConstPtr const & imu, nav_msgs::OdometryConstPtr const & odom ){

  if ( !cali_sampler.enough_samples() ) {
    cali_sampler.sample(imu,odom);
  }

  if ( cali_sampler.enough_samples() ) {
    ROS_INFO_STREAM("DO CERES CALIBRATION");
    calibrate_vicon_imu(cali_sampler);
    ros::shutdown();
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