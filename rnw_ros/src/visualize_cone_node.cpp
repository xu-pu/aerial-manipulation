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

    bool contact_valid = false;
    Vector3d contact_point;

    static constexpr double min_tilt = 5;

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
      calc_contact_point();
    }

    void calc_contact_point(){

      double tilt_x = abs(asin(R_markers(2,0)));
      double tilt_x_deg = tilt_x / M_PI * 180;
      if ( tilt_x_deg < min_tilt ) {
        ROS_WARN_STREAM("not point contact, " << tilt_x_deg );
        contact_valid = false;
        return;
      }

      double x0 = T_center.x();
      double y0 = T_center.y();
      double z0 = T_center.z();

      double zg = rnw_config.ground_z;

      Vector3d n = R_markers.col(2);

      double A = n.x();
      double B = n.y();
      double C = n.z()*(zg-z0) - A*x0 - B*y0;

      double dist_2d = abs(A*x0 + B*y0 + C) / sqrt(A*A + B*B);
      double dist = sqrt(dist_2d*dist_2d+z0*z0);

      double ratio = dist/rnw_config.cone.radius;

      if ( ratio > 1.1 ) {
        // lifted off the ground
        ROS_WARN_STREAM("lifted off the ground");
        contact_valid = false;
        return;
      }

      double lambda = - (A*x0 + B * y0 + C) / (A * A + B * B);

      Vector3d pt = { x0+lambda*A, y0 + lambda * B, rnw_config.ground_z };

      contact_point = pt;

      contact_valid = true;

    }

};

struct cone_visualizer_t {

    double clear_after_n_sec = numeric_limits<double>::max();

    obj_state_estimator_t estimator;

    ros::Time latest_time;

    bool init = false;

    ros::Publisher pub_marker_cone;

    ros::Publisher pub_contact_point;

    static constexpr int id_base = 0;

    static constexpr int id_shaft = 1;

    static constexpr int id_contact_path = 2;

    string ns = "cone_state_visualization";

    double cone_color_r = 0;
    double cone_color_g = 0;
    double cone_color_b = 0;

    visualization_msgs::Marker marker_contact_path;

    explicit cone_visualizer_t( ros::NodeHandle & nh ) : estimator(nh) {
      pub_marker_cone = nh.advertise<visualization_msgs::MarkerArray>("markers/cone", 1);
      pub_contact_point = nh.advertise<geometry_msgs::PointStamped>("/rnw/ground_contact_point", 10);
      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",numeric_limits<double>::max());
      cone_color_r = get_param_default(nh,"cone_color_r",0);
      cone_color_g = get_param_default(nh,"cone_color_g",0);
      cone_color_b = get_param_default(nh,"cone_color_b",0);

      init_marker_contact_path();

    }

    void init_marker_contact_path(){
      marker_contact_path.id = id_contact_path;
      marker_contact_path.type = visualization_msgs::Marker::LINE_STRIP;
      marker_contact_path.header.stamp = ros::Time::now();
      marker_contact_path.header.frame_id = "world";
      marker_contact_path.action = visualization_msgs::Marker::ADD;
      marker_contact_path.ns = ns;
      marker_contact_path.color.r = 1;
      marker_contact_path.color.g = 0;
      marker_contact_path.color.b = 0;
      marker_contact_path.color.a = 1.00;
      marker_contact_path.pose.orientation.w = 1;
      marker_contact_path.scale.x = 0.01;
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg  ){
      latest_time = ros::Time::now();
      estimator.update(*msg);
      pub_marker_cone.publish(gen_markers());
      if ( estimator.contact_valid ) {
        geometry_msgs::PointStamped pt_msg;
        pt_msg.header = msg->header;
        pt_msg.point = uav_utils::to_point_msg(estimator.contact_point);
        pub_contact_point.publish(pt_msg);
      }
      init = true;
    }

    visualization_msgs::MarkerArray gen_markers(){
      visualization_msgs::MarkerArray marker_arr;
      marker_arr.markers.push_back(gen_marker_base());
      marker_arr.markers.push_back(gen_marker_shaft());
      marker_arr.markers.push_back(gen_contact_path());
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

    visualization_msgs::Marker gen_contact_path(){
      if ( estimator.contact_valid ) {
        marker_contact_path.points.push_back(uav_utils::to_point_msg(estimator.contact_point));
      }
      return marker_contact_path;
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