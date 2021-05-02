#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

template<typename T>
T get_param_default( ros::NodeHandle & nh, string const & key, T const & default_val ){
  T val;
  if ( !nh.getParam(key,val) ) {
    val = default_val;
  }
  return val;
}

struct uav_odom_filter_t {

    bool init_ok = false;

    Vector3d    init_P, last_P;
    Quaterniond init_Q, last_Q, vins_Q;
    ros::Time   now_t, last_odom_t;
    Vector3d    Vi0, Vi1, Vi2, Vi3, Vi4, Vo0;

    Quaterniond R_MARKER_FLU;
    Vector3d T_MARKER_FLU;

    uav_odom_filter_t(){
      T_MARKER_FLU = Vector3d::Zero();
      R_MARKER_FLU = Matrix3d::Identity();
      Vi0.setZero();
      Vi1.setZero();
      Vi2.setZero();
      Vi3.setZero();
      Vi4.setZero();
      Vo0.setZero();
    }

    nav_msgs::Odometry update( geometry_msgs::PoseStampedConstPtr const & msg ) {

      auto const &msg_quat = msg->pose.orientation;
      auto const &msg_pos = msg->pose.position;

      Quaterniond R_VICON_MARKER = Quaterniond(msg_quat.w, msg_quat.x, msg_quat.y, msg_quat.z).normalized();
      Vector3d T_VICON_MARKER(msg_pos.x, msg_pos.y, msg_pos.z);

      Quaterniond R_VICON_FLU = R_VICON_MARKER * R_MARKER_FLU;
      Vector3d T_VICON_FLU = R_VICON_MARKER * T_MARKER_FLU + T_VICON_MARKER;

      Vector3d now_P, P_w;
      Quaterniond now_Q, Q_w;

      now_Q = R_VICON_FLU;
      now_P = T_VICON_FLU;

      Q_w = now_Q.normalized().toRotationMatrix();
      P_w = now_P;

      if (!init_ok) {
        init_ok = true;
        init_Q = R_VICON_FLU;
        init_P = T_VICON_FLU;
        last_P = init_P;
        last_Q = init_Q;
        last_odom_t = msg->header.stamp;
      }
      else {
        now_t = msg->header.stamp;

        Vector3d now_vel;

        if ((now_t - last_odom_t).toSec() > 0.001) {
          now_vel = (P_w - last_P) / (now_t - last_odom_t).toSec();
          //        std::cout << " time " << ( now_t - last_t ).toSec( ) << std::endl;
          //        std::cout << " now_vel " << now_vel << std::endl;

          /** velocity filter **/
          if ((now_vel - Vi0).norm() / (now_t - last_odom_t).toSec() > 20.0) {
            //printf("Vel error\n");
          } else {
            Vi0 = now_vel;
            Vo0 = (Vi0 + Vi1 + Vi2 + Vi3 + Vi4) * 0.2;
            Vi4 = Vi3;
            Vi3 = Vi2;
            Vi2 = Vi1;
            Vi1 = Vi0;
            last_odom_t = now_t;
            last_P = P_w;
            last_Q = Q_w;
          }
        }
      }

      /*********************/

      nav_msgs::Odometry odom;
      odom.header.stamp = now_t;
      odom.header.frame_id = "world";
      odom.pose.pose.position.x = P_w.x();
      odom.pose.pose.position.y = P_w.y();
      odom.pose.pose.position.z = P_w.z();
      odom.pose.pose.orientation.w = Q_w.w();
      odom.pose.pose.orientation.x = Q_w.x();
      odom.pose.pose.orientation.y = Q_w.y();
      odom.pose.pose.orientation.z = Q_w.z();
      odom.twist.twist.linear.x = Vo0.x(); // now_vel.x();
      odom.twist.twist.linear.y = Vo0.y(); // now_vel.y();
      odom.twist.twist.linear.z = Vo0.z(); // now_vel.z();

      return odom;

    }
};

struct mocap_processor_t {

    ros::NodeHandle & nh;

    ros::Publisher pub_odom_cone;
    ros::Publisher pub_odom_drone1;
    ros::Publisher pub_odom_drone2;

    ros::Subscriber sub_drone_1;
    ros::Subscriber sub_drone_2;
    ros::Subscriber sub_cone;

    uav_odom_filter_t drone1;
    uav_odom_filter_t drone2;

    explicit mocap_processor_t( ros::NodeHandle & node_handle ) : nh(node_handle) {

      /**
       * Load Vicon-IMU Calibration
       */

      drone1.R_MARKER_FLU.setIdentity();
      drone1.T_MARKER_FLU.x() = get_param_default(nh,"drone1/T_MARKER_FLU/x",0.);
      drone1.T_MARKER_FLU.y() = get_param_default(nh,"drone1/T_MARKER_FLU/y",0.);
      drone1.T_MARKER_FLU.z() = get_param_default(nh,"drone1/T_MARKER_FLU/z",0.);

      drone2.R_MARKER_FLU.setIdentity();
      drone2.T_MARKER_FLU.x() = get_param_default(nh,"drone2/T_MARKER_FLU/x",0.);
      drone2.T_MARKER_FLU.y() = get_param_default(nh,"drone2/T_MARKER_FLU/y",0.);
      drone2.T_MARKER_FLU.z() = get_param_default(nh,"drone2/T_MARKER_FLU/z",0.);

      ROS_INFO_STREAM("Extrinsics:");
      ROS_INFO_STREAM("Drone #1 T_MARKER_FLU: " << drone1.T_MARKER_FLU.transpose());
      ROS_INFO_STREAM("Drone #2 T_MARKER_FLU: " << drone2.T_MARKER_FLU.transpose());

      bool publish_uav_odom = get_param_default<bool>(nh, "publish_uav_odom", true);

      if ( publish_uav_odom ) {

        pub_odom_drone1 = nh.advertise<nav_msgs::Odometry>("/drone1/odom", 100);

        pub_odom_drone2 = nh.advertise<nav_msgs::Odometry>("/drone2/odom", 100);

        sub_drone_1 = nh.subscribe<geometry_msgs::PoseStamped>(
                "/mocap/drone1",
                100,
                &mocap_processor_t::drone1_pose_callback,
                this,
                ros::TransportHints().tcpNoDelay()
        );

        sub_drone_2 = nh.subscribe<geometry_msgs::PoseStamped>(
                "/mocap/drone2",
                100,
                &mocap_processor_t::drone2_pose_callback,
                this,
                ros::TransportHints().tcpNoDelay()
        );

      }

      pub_odom_cone = nh.advertise<nav_msgs::Odometry>("/cone/odom", 100);

      sub_cone = nh.subscribe<geometry_msgs::PoseStamped>(
              "/mocap/cone",
              100,
              &mocap_processor_t::pose_cone_callback,
              this,
              ros::TransportHints().tcpNoDelay()
      );

    }

    void drone1_pose_callback(geometry_msgs::PoseStampedConstPtr const & msg ) {
      nav_msgs::Odometry odom = drone1.update(msg);
      pub_odom_drone1.publish(odom);
    }

    void drone2_pose_callback(geometry_msgs::PoseStampedConstPtr const & msg ) {
      nav_msgs::Odometry odom = drone2.update(msg);
      pub_odom_drone2.publish(odom);
    }

    void pose_cone_callback( geometry_msgs::PoseStampedConstPtr const & msg ) const {

      Vector3d    now_P, P_w;
      Quaterniond now_Q, Q_w;
      now_P.x() = msg->pose.position.x;
      now_P.y() = msg->pose.position.y;
      now_P.z() = msg->pose.position.z;
      now_Q.w() = msg->pose.orientation.w;
      now_Q.x() = msg->pose.orientation.x;
      now_Q.y() = msg->pose.orientation.y;
      now_Q.z() = msg->pose.orientation.z;

      Q_w = now_Q.normalized().toRotationMatrix();
      P_w = now_P;

      /*********************/

      nav_msgs::Odometry odom;
      odom.header.stamp = msg->header.stamp;
      odom.header.frame_id = "world";
      odom.pose.pose.position.x = P_w.x();
      odom.pose.pose.position.y = P_w.y();
      odom.pose.pose.position.z = P_w.z();
      odom.pose.pose.orientation.w = Q_w.w();
      odom.pose.pose.orientation.x = Q_w.x();
      odom.pose.pose.orientation.y = Q_w.y();
      odom.pose.pose.orientation.z = Q_w.z();
      pub_odom_cone.publish( odom );

    }

};

int main( int argc, char **argv ){

  ros::init( argc, argv, "mocap2odom" );

  ros::NodeHandle n( "~" );

  mocap_processor_t mocap_processor(n);

  ros::spin();

  ros::shutdown();

  return 0;

}
