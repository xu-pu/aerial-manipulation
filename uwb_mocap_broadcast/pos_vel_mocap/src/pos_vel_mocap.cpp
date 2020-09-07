#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pos_vel_mocap/ViconCalibConfig.h>

#define IF_SUBTRACT_INIT 0
using namespace std;
using namespace Eigen;

ros::Publisher pub_odom_uav;
ros::Publisher pub_odom_cone;
ros::Publisher pub_path;

bool init_ok = false;

Eigen::Vector3d    init_P, last_P;
Eigen::Quaterniond init_Q, last_Q, vins_Q;
ros::Time          now_t, last_odom_t, last_path_t;
Eigen::Vector3d    Vi0, Vi1, Vi2, Vi3, Vi4, Vo0;

nav_msgs::Path run_path;

Eigen::Quaterniond R_MARKER_FLU;
Eigen::Vector3d T_MARKER_FLU;

Vector3d rpy_markers;

static constexpr double DEG2RAD = M_PI/180;

inline Eigen::Matrix3d rpy2rot( Vector3d const & rpy ){
  return Eigen::AngleAxisd(rpy.z(), Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Vector3d::UnitX())
                 .toRotationMatrix();
}

void calib_cfg_callback(pos_vel_mocap::ViconCalibConfig & config, uint32_t level ){

  rpy_markers.x() = DEG2RAD * config.yaw;
  rpy_markers.y() = DEG2RAD * config.pitch;
  rpy_markers.z() = DEG2RAD * config.roll;

  Matrix3d R_FLU_MARKER = rpy2rot(rpy_markers);

  R_MARKER_FLU = R_FLU_MARKER.transpose();

}

void pose_callback( const geometry_msgs::PoseStamped::ConstPtr msg ) {

  auto const & msg_quat = msg->pose.orientation;
  auto const & msg_pos = msg->pose.position;

  Quaterniond R_VICON_MARKER = Quaterniond(msg_quat.w,msg_quat.x,msg_quat.y,msg_quat.z).normalized();
  Vector3d T_VICON_MARKER(msg_pos.x,msg_pos.y,msg_pos.z);

  Quaterniond R_VICON_FLU = R_VICON_MARKER * R_MARKER_FLU;
  Vector3d T_VICON_FLU = R_VICON_MARKER * T_MARKER_FLU + T_VICON_MARKER;

    if ( !init_ok )
    {
        init_ok = true;
        init_Q = R_VICON_FLU;
        init_P = T_VICON_FLU;
        last_P = init_P;
        last_Q = init_Q;
        last_odom_t = msg->header.stamp;
    }
    else
    {
        now_t = msg->header.stamp;

        Eigen::Vector3d    now_P, P_w;
        Eigen::Quaterniond now_Q, Q_w;

        now_Q = R_VICON_FLU;
        now_P = T_VICON_FLU;

        //std::cout << "x :" << msg->pose.position.x << "y:" << msg->pose.position.y
        //          << " z :" << msg->pose.position.z << std::endl;
        // Q_w = init_Q.normalized().toRotationMatrix().transpose() *
        // now_Q.normalized().toRotationMatrix();
        Q_w = now_Q.normalized().toRotationMatrix();
        if(IF_SUBTRACT_INIT)
        {
            P_w = now_P - init_P;
        }
        else
        {
            P_w = now_P;
        }

        Eigen::Vector3d now_vel;

        if((now_t-last_odom_t).toSec()>0.001)
        {
            now_vel = ( P_w - last_P ) / ( now_t - last_odom_t ).toSec();
            //        std::cout << " time " << ( now_t - last_t ).toSec( ) << std::endl;
            //        std::cout << " now_vel " << now_vel << std::endl;

            /** velocity filter **/
            if( (now_vel - Vi0).norm() / (now_t-last_odom_t).toSec() > 20.0 )
            {
                //printf("Vel error\n");
            }
            else
            {
                Vi0 = now_vel;
                Vo0 = ( Vi0 + Vi1 + Vi2 + Vi3 + Vi4 ) * 0.2;
                Vi4 = Vi3;
                Vi3 = Vi2;
                Vi2 = Vi1;
                Vi1 = Vi0;
                last_odom_t = now_t;
                last_P = P_w;
                last_Q = Q_w;
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
        pub_odom_uav.publish(odom );


//        ros::Duration delta_t = now_t - last_path_t;
//        if ( delta_t.toSec() > 0.1 )
//        {
//            geometry_msgs::PoseStamped pose;
//            pose.header.stamp = now_t;
//            pose.header.frame_id = "world";
//            pose.pose.orientation.x = odom.pose.pose.orientation.x;
//            pose.pose.orientation.y = odom.pose.pose.orientation.y;
//            pose.pose.orientation.z = odom.pose.pose.orientation.z;
//            pose.pose.orientation.w = odom.pose.pose.orientation.w;
//            pose.pose.position.x = odom.pose.pose.position.x;
//            pose.pose.position.y = odom.pose.pose.position.y;
//            pose.pose.position.z = odom.pose.pose.position.z;

//            run_path.header.stamp = now_t;
//            run_path.header.frame_id = "world";
//            run_path.poses.push_back( pose );
//            pub_path.publish( run_path );

//            last_path_t = now_t;
//        }
    }
}

void pose_cone_callback( const geometry_msgs::PoseStamped::ConstPtr msg ) {

  Eigen::Vector3d    now_P, P_w;
  Eigen::Quaterniond now_Q, Q_w;
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

template<typename T>
T get_param_default( ros::NodeHandle & nh, string const & key, T const & default_val ){
  T val;
  if ( !nh.getParam(key,val) ) {
    val = default_val;
  }
  return val;
}

int main( int argc, char **argv ){

    ros::init( argc, argv, "pos_vel_mocap" );
    ros::NodeHandle n( "~" );

    rpy_markers.setZero();
    R_MARKER_FLU.setIdentity();

    T_MARKER_FLU.x() = get_param_default(n,"T_MARKER_FLU/x",0.);
    T_MARKER_FLU.y() = get_param_default(n,"T_MARKER_FLU/y",0.);
    T_MARKER_FLU.z() = get_param_default(n,"T_MARKER_FLU/z",0.);

    ROS_INFO_STREAM("Load T_MARKER_FLU: " << T_MARKER_FLU.transpose());

  dynamic_reconfigure::Server<pos_vel_mocap::ViconCalibConfig> server;
  dynamic_reconfigure::Server<pos_vel_mocap::ViconCalibConfig>::CallbackType f;
  server.setCallback(calib_cfg_callback);

  bool publish_uav_odom = get_param_default<bool>(n, "publish_uav_odom", true);

  ros::Subscriber sub_uav;

  if(publish_uav_odom){

    sub_uav = n.subscribe<geometry_msgs::PoseStamped>(
            "/mocap/uav",
            100,
            pose_callback,
            nullptr,
            ros::TransportHints().tcpNoDelay()
    );

    pub_odom_uav = n.advertise<nav_msgs::Odometry>("/odom/uav", 100);

  }

  ros::Subscriber sub_cone = n.subscribe<geometry_msgs::PoseStamped>(
          "/mocap/cone",
          100,
          pose_cone_callback,
          nullptr,
          ros::TransportHints().tcpNoDelay()
  );

  pub_odom_cone = n.advertise<nav_msgs::Odometry>("/odom/cone", 100);

  ros::spin();

  ros::shutdown();

  return 0;

}
