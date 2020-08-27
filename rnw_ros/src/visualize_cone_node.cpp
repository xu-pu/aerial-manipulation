#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"

#include <uav_utils/converters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

struct obj_state_estimator_t {

    rnw_config_t rnw_config;

    Vector3d X_base_body;

    Vector3d X_center_body;

    nav_msgs::Odometry latest_odom;

    double diameter() const {
      return rnw_config.cone.radius * 2;
    }

    // object state vars

    Matrix3d R_markers;
    Vector3d T_markers;

    Vector3d T_tip;
    Vector3d T_base;
    Vector3d T_center;

    explicit obj_state_estimator_t( ros::NodeHandle & nh ) {
      rnw_config.load_from_ros(nh);
      X_base_body = rnw_config.X_tip_body;
      X_base_body.z() = rnw_config.X_tip_body.z() - rnw_config.cone.height;
      X_center_body = X_base_body;
      X_center_body.x() -= rnw_config.cone.radius;
    }

    void update( nav_msgs::Odometry const & odom ){
      latest_odom = odom;
      R_markers = odom2R(odom);
      T_markers = odom2T(odom);
      T_tip = R_markers * rnw_config.X_tip_body + T_markers;
      T_base = R_markers * X_base_body + T_markers;
      T_center = R_markers * X_center_body + T_markers;
    }

};

struct cone_visualizer_t {

    double clear_after_n_sec = numeric_limits<double>::max();

    obj_state_estimator_t estimator;

    ros::Time latest_time;

    bool init = false;

    ros::Publisher pub_marker_cone;

    static constexpr int id_base = 0;

    static constexpr int id_shaft = 1;

    string ns = "cone_state_visualization";

    double cone_color_r = 0;
    double cone_color_g = 0;
    double cone_color_b = 0;

    explicit cone_visualizer_t( ros::NodeHandle & nh ) : estimator(nh) {
      pub_marker_cone = nh.advertise<visualization_msgs::MarkerArray>("markers/cone", 1);
      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",numeric_limits<double>::max());
      cone_color_r = get_param_default(nh,"cone_color_r",0);
      cone_color_g = get_param_default(nh,"cone_color_g",0);
      cone_color_b = get_param_default(nh,"cone_color_b",0);
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg  ){
      latest_time = ros::Time::now();
      estimator.update(*msg);
      pub_marker_cone.publish(gen_markers());
      init = true;
    }

    visualization_msgs::MarkerArray gen_markers(){
      visualization_msgs::MarkerArray marker_arr;
      marker_arr.markers.push_back(gen_marker_base());
      marker_arr.markers.push_back(gen_marker_shaft());
      return marker_arr;
    }

    visualization_msgs::Marker gen_marker_shaft() const {

      visualization_msgs::Marker marker;

      marker.id = id_shaft;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = ns;
      marker.color.r = cone_color_r;
      marker.color.g = cone_color_g;
      marker.color.b = cone_color_b;
      marker.color.a = 1.00;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.01;
      marker.points.push_back(uav_utils::to_point_msg(estimator.T_base));
      marker.points.push_back(uav_utils::to_point_msg(estimator.T_tip));
      return marker;
    }

    visualization_msgs::Marker gen_marker_base() const {

      visualization_msgs::Marker marker;

      marker.id = id_base;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = ns;
      marker.color.r = cone_color_r;
      marker.color.g = cone_color_g;
      marker.color.b = cone_color_b;
      marker.color.a = 1.00;

      marker.pose.orientation = estimator.latest_odom.pose.pose.orientation;
      marker.pose.position = uav_utils::to_point_msg(estimator.T_center);
      marker.scale.x = estimator.diameter();
      marker.scale.y = estimator.diameter();
      marker.scale.z = 0.01;

      return marker;

    }

    void clear_markers() const {
      visualization_msgs::MarkerArray accMarkers;
      visualization_msgs::Marker accMarker;
      accMarker.action = visualization_msgs::Marker::DELETEALL;
      accMarkers.markers.push_back(accMarker);
      pub_marker_cone.publish(accMarkers);
    }

    void on_spin( const ros::TimerEvent &event ){
      if ( !init ) { return; }
      if ( (ros::Time::now() - latest_time).toSec() > clear_after_n_sec ) {
        clear_markers();
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_cone_node");

  ros::NodeHandle nh("~");

  cone_visualizer_t cone_viz(nh);

  constexpr size_t spin_hz = 10;

  auto timer = nh.createTimer( ros::Duration( 1.0 / spin_hz ), &cone_visualizer_t::on_spin, &cone_viz );

  ros::Subscriber sub_traj = nh.subscribe<nav_msgs::Odometry>("odom", 100, &cone_visualizer_t::on_odom, &cone_viz );

  ros::spin();

  ros::shutdown();

  return 0;

}