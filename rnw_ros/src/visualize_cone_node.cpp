#include "rnw_ros/pose_utils.h"

#include <uav_utils/converters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

using Eigen::Vector3d;


struct obj_state_estimator_t {

    double radius = 0.15;
    double height = 1;
    Vector3d X_tip_body;
    Vector3d X_base_body;
    Vector3d X_center_body;

    nav_msgs::Odometry latest_odom;

    double diameter() const {
      return radius * 2;
    }

    // object state vars

    Matrix3d R_markers;
    Vector3d T_markers;

    Vector3d T_tip;
    Vector3d T_base;
    Vector3d T_center;

    obj_state_estimator_t() {
      X_tip_body =  { 0.00922753,0.00883189,0.659304 };
      X_base_body = X_tip_body;
      X_base_body.z() = X_tip_body.z() - height;
      X_center_body = X_base_body;
      X_center_body.x() -= radius;
    }

    void update( nav_msgs::Odometry const & odom ){
      latest_odom = odom;
      R_markers = odom2R(odom);
      T_markers = odom2T(odom);
      T_tip = R_markers * X_tip_body + T_markers;
      T_base = R_markers * X_base_body + T_markers;
      T_center = R_markers * X_center_body + T_markers;
    }

};

struct cone_visualizer_t {

    double clear_after_n_sec = numeric_limits<double>::max();

    /**
     * record the receiving time using ros::Time::now() so it works in rosbag playback
     */
    ros::Time latest_time;

    nav_msgs::Odometry latest_odom;

    obj_state_estimator_t estimator;

    bool init = false;

    ros::Publisher pub_marker_cone;

    explicit cone_visualizer_t( ros::NodeHandle & nh ) {
      pub_marker_cone = nh.advertise<visualization_msgs::Marker>("markers/cone", 1);
      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",numeric_limits<double>::max());
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg  ){
      latest_odom = *msg;
      latest_time = ros::Time::now();
      estimator.update(*msg);
      pub_marker_cone.publish(gen_marker_base());
      init = true;
    }

    visualization_msgs::Marker gen_marker_base() const {

      constexpr int id = 0;

      visualization_msgs::Marker marker;

      marker.id = id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "rnw";
      marker.color.r = 0.00;
      marker.color.g = 1.00;
      marker.color.b = 0.00;
      marker.color.a = 1.00;

      marker.pose.orientation = estimator.latest_odom.pose.pose.orientation;
      marker.pose.position = uav_utils::to_point_msg(estimator.T_center);
      marker.scale.x = estimator.diameter();
      marker.scale.y = estimator.diameter();
      marker.scale.z = 0.01;

      return marker;

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_cone_node");

  ros::NodeHandle nh("~");

  cone_visualizer_t cone_viz(nh);

  constexpr size_t spin_hz = 10;

  //auto timer = nh.createTimer( ros::Duration( 1.0 / spin_hz ), &cone_visualizer_t::on_spin, &cone_viz );

  ros::Subscriber sub_traj = nh.subscribe<nav_msgs::Odometry>("/pos_vel_mocap/odom_cone", 100, &cone_visualizer_t::on_odom, &cone_viz );

  ros::spin();

  ros::shutdown();

  return 0;

}