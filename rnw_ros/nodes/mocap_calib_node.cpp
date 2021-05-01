#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <uav_utils/converters.h>

#include <dynamic_reconfigure/server.h>
#include <rnw_ros/MocapCalibConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "rnw_ros/pose_utils.h"
#include "rnw_ros/traj_uitls.h"

Vector3d from_vicon( Vector3d const & pt ){
  // OptiTrack's cord is in (x,z,-y)
  return { pt.x(), -pt.z(), pt.y() };
}

struct mocap_calibrator_t {

    geometry_msgs::PoseStamped latest_pose_cone;
    geometry_msgs::PoseStamped latest_pose_uav;

    Matrix3d uav_R;
    Vector3d uav_T;

    Matrix3d cone_R;
    Vector3d cone_T;

    void on_odom_uav( geometry_msgs::PoseStampedConstPtr const & msg ){
      latest_pose_uav = *msg;
      Matrix3d R = uav_utils::from_quaternion_msg(msg->pose.orientation).toRotationMatrix();
      Vector3d T = uav_utils::from_point_msg(msg->pose.position);
      uav_R = R.transpose();
      uav_T = - uav_R * T;
    }

    void on_odom_cone( geometry_msgs::PoseStampedConstPtr const & msg ){
      latest_pose_cone = *msg;
      Matrix3d R = uav_utils::from_quaternion_msg(msg->pose.orientation).toRotationMatrix();
      Vector3d T = uav_utils::from_point_msg(msg->pose.position);
      cone_R = R.transpose();
      cone_T = - cone_R * T;
    }

    void cfg_callback( rnw_ros::MocapCalibConfig & config, uint32_t level ){

      ROS_INFO_STREAM("cfg callback");

      // to eigen

      Vector3d cage_center( config.cage_center_x, config.cage_center_y, config.cage_center_z );
      Vector3d base_center( config.base_center_x, config.base_center_y, config.base_center_z );
      Vector3d tip( config.Tip_x, config.Tip_y, config.Tip_z );

      // correct vicon point order

      cage_center = from_vicon(cage_center);
      base_center = from_vicon(base_center);
      tip = from_vicon(tip);

      // to body frame

      cage_center = uav_R * cage_center + uav_T;

      base_center = cone_R * base_center + cone_T;
      tip = cone_R * tip + cone_T;

      // calc

      cout << "===================================\n";

      cout << "X_tcp_cage:\n"
           << "  x: " << cage_center.x() << '\n'
           << "  y: " << cage_center.y() << '\n'
           << "  z: " << cage_center.z() << '\n'
           << "cone:\n"
           << "  base_center:\n"
           << "    x: " << base_center.x() << '\n'
           << "    y: " << base_center.y() << '\n'
           << "    z: " << base_center.z() << '\n'
           << "  tip:\n"
           << "    x: " << tip.x() << '\n'
           << "    y: " << tip.y() << '\n'
           << "    z: " << tip.z() << endl;

      cout << "===================================\n";

      cout << endl;

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"mocap_calib_node");

  ros::NodeHandle nh("~");

  mocap_calibrator_t cali;

  ros::Subscriber sub_pose_uav = nh.subscribe<geometry_msgs::PoseStamped>(
          "uav",
          10,
          &mocap_calibrator_t::on_odom_uav,
          &cali,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_pose_cone = nh.subscribe<geometry_msgs::PoseStamped>(
          "cone",
          10,
          &mocap_calibrator_t::on_odom_cone,
          &cali,
          ros::TransportHints().tcpNoDelay()
  );

  ROS_INFO_STREAM("rosrun rqt_reconfigure rqt_reconfigure");
  ROS_INFO_STREAM("x, y, z is in the order OptiTrack shows, we will handle it");

  dynamic_reconfigure::Server<rnw_ros::MocapCalibConfig> server;
  server.setCallback([&]( rnw_ros::MocapCalibConfig & config, uint32_t level ){
      cali.cfg_callback(config,level);
  });

  ros::spin();

  ros::shutdown();

  return 0;

}