#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"

#include <uav_utils/converters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

struct cone_visualizer_t {

    double clear_after_n_sec = numeric_limits<double>::max();

    cone_state_estimator_t estimator;

    ros::Time latest_time;

    bool init = false;

    ros::Publisher pub_marker_cone;

    ros::Publisher pub_contact_point;

    static constexpr int id_base = 0;

    static constexpr int id_shaft = 1;

    static constexpr int id_contact_path = 2;

    static constexpr int id_contact_normal = 3;

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
      estimator.on_odom(msg);
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
      marker_arr.markers.push_back(gen_marker_contact_normal());
      return marker_arr;
    }

    visualization_msgs::Marker gen_marker_contact_normal() const {

      visualization_msgs::Marker marker;

      marker.id = id_contact_normal;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.ns = ns;
      marker.scale.x = 0.01;
      marker.scale.y = 0.03;
      marker.scale.z = 0.1;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1.00;
      marker.pose.orientation.w = 1;

      if ( estimator.contact_valid ) {
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.push_back(uav_utils::to_point_msg(estimator.contact_point));
        marker.points.push_back(uav_utils::to_point_msg(estimator.contact_point+Vector3d(0,0,0.5)));
      } else {
        marker.action = visualization_msgs::Marker::DELETE;
      }

      return marker;

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

      marker.pose.orientation = estimator.previous_odom.pose.pose.orientation;
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