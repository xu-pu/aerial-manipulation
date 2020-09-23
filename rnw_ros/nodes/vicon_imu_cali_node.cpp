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

struct cali_sampler_t {

    vector<Vector3d> samples_rpy;
    vector<sensor_msgs::Imu> samples_imu;
    vector<nav_msgs::Odometry> samples_odom;

    size_t sample_amount = 500;

    double sample_interval_deg = 1;

    explicit cali_sampler_t( ros::NodeHandle & nh ){
      sample_amount = get_param_default<int>(nh, "sample_amount", 500);
      sample_interval_deg = get_param_default<double>(nh, "sample_interval_deg", 1);
    }

    bool check_rpy( Vector3d const & rpy ){

      for ( auto const & pt : samples_rpy ) {

        double dist = (rpy-pt).norm();

        if ( dist < sample_interval_deg * deg2rad ) {
          return false;
        }

      }

      return true;

    }

    void sample( ImuConstPtr const & imu, OdometryConstPtr const & odom ){

      Vector3d rpy = imu2rpy(imu);

      if ( check_rpy(rpy) ) {
        ROS_INFO_STREAM("sync offset: " << int(abs((imu->header.stamp - odom->header.stamp).toSec())*1000.) << "ms");
        samples_rpy.push_back(rpy);
        samples_imu.push_back(*imu);
        samples_odom.push_back(*odom);
        ROS_INFO_STREAM("Samples Collected: " << samples_rpy.size() << "/" << sample_amount);
      }

    }

    bool enough_samples() const {
      return samples_rpy.size() > sample_amount;
    }

    void sync_callback( sensor_msgs::ImuConstPtr const & imu, nav_msgs::OdometryConstPtr const & odom ){

      if ( !enough_samples() ) {
        sample(imu,odom);
      }

      if ( enough_samples() ) {
        ROS_INFO_STREAM("DO CERES CALIBRATION");
        calibrate_vicon_imu();
        ros::shutdown();
      }

    }

    void calibrate_vicon_imu(){

      using namespace ceres;

      assert( enough_samples() );

      ceres::Problem problem;

      Quaterniond R_MARKER_FLU = Quaterniond::Identity();

      double * data = R_MARKER_FLU.coeffs().data();

      for ( size_t i=0; i<samples_rpy.size(); i++ ) {

        auto const & imu = samples_imu.at(i);
        auto const & odom = samples_odom.at(i);

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
      ROS_INFO_STREAM("Eigen::Quat: " << R_MARKER_FLU.w() << ", " << R_MARKER_FLU.x() << ", " << R_MARKER_FLU.y() << ", " << R_MARKER_FLU.z());
      ROS_INFO_STREAM("Magnitude: " << rad2deg * Eigen::AngleAxisd(R_MARKER_FLU).angle() );

      cout << "yaml:" << endl;
      cout << "R_MARKER_FLU:\n"
           << "   x: " << R_MARKER_FLU.x() << '\n'
           << "   y: " << R_MARKER_FLU.y() << '\n'
           << "   z: " << R_MARKER_FLU.z() << '\n'
           << "   w: " << R_MARKER_FLU.w() << '\n' << endl;

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"vicon_imu_calib_node");

  ros::NodeHandle nh("~");

  cali_sampler_t cali_sampler(nh);

  message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, "imu", 1, ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, "odom", 1, ros::TransportHints().tcpNoDelay());
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,nav_msgs::Odometry> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10),sub_imu,sub_odom);
  sync.registerCallback(&cali_sampler_t::sync_callback,&cali_sampler);

  ros::spin();

  return 0;

}