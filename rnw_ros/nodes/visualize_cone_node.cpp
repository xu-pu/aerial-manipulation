#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_config.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RnwState.h>

using namespace std;

struct cone_visualizer_t {

    rnw_config_t rnw_cfg;

    ros::Publisher pub_marker_cone;

    rnw_msgs::ConeState latest_cone_state;
    rnw_msgs::RnwState latest_rnw_state;

    ros::Subscriber sub_cone_state;
    ros::Subscriber sub_rnw_state;

    bool cone_state_ontime() const {
      return (ros::Time::now() - latest_cone_state.header.stamp).toSec() < 0.5;
    }

    bool rnw_cmd_ontime() const {
      return (ros::Time::now() - latest_rnw_state.header.stamp).toSec() < 0.5;
    }

    static constexpr int id_base = 0;
    static constexpr int id_shaft = 1;
    static constexpr int id_contact_path = 2;
    static constexpr int id_contact_normal = 3;
    static constexpr int id_rocking_cmd = 4;
    static constexpr int id_grip = 5;
    static constexpr int id_moai = 6;

    string ns = "cone_state_visualization";

    double cone_color_r = 0;
    double cone_color_g = 0;
    double cone_color_b = 0;

    visualization_msgs::Marker marker_contact_path;

    std_msgs::ColorRGBA color_qstatic;
    std_msgs::ColorRGBA color_default;

    std_msgs::ColorRGBA color;

    explicit cone_visualizer_t( ros::NodeHandle & nh ) {

      rnw_cfg.load_from_ros(nh);

      cone_color_r = get_param_default(nh,"cone_color_r",0);
      cone_color_g = get_param_default(nh,"cone_color_g",0);
      cone_color_b = get_param_default(nh,"cone_color_b",0);

      color_qstatic.r = 1;
      color_qstatic.g = 0;
      color_qstatic.b = 0;
      color_qstatic.a = 1;

      color_default.r = 0;
      color_default.g = 1;
      color_default.b = 0;
      color_default.a = 1;

      init_marker_contact_path();

      pub_marker_cone = nh.advertise<visualization_msgs::MarkerArray>("markers/cone", 1);

      sub_cone_state = nh.subscribe<rnw_msgs::ConeState>(
              "/cone/state",
              100,
              &cone_visualizer_t::on_cone_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_rnw_state = nh.subscribe<rnw_msgs::RnwState>(
              "/rnw/state",
              100,
              &cone_visualizer_t::on_rnw_state,
              this,
              ros::TransportHints().tcpNoDelay()
      );

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

    void on_cone_state( rnw_msgs::ConeStateConstPtr const & msg  ){
      latest_cone_state = *msg;
      color = color_default;
      pub_marker_cone.publish(gen_markers());
    }

    void on_rnw_state( rnw_msgs::RnwStateConstPtr const & msg ){
      latest_rnw_state = *msg;
    }

    visualization_msgs::MarkerArray gen_markers(){
      visualization_msgs::MarkerArray marker_arr;
      marker_arr.markers.push_back(gen_marker_base());
      marker_arr.markers.push_back(gen_marker_shaft());
      marker_arr.markers.push_back(gen_contact_path());
      marker_arr.markers.push_back(gen_marker_contact_normal());
      marker_arr.markers.push_back(gen_marker_rnw_cmd());
      //marker_arr.markers.push_back(gen_marker_moai());
      return marker_arr;
    }

    visualization_msgs::Marker gen_marker_rnw_cmd() const {

      visualization_msgs::Marker marker;

      marker.id = id_rocking_cmd;
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

      marker.action = visualization_msgs::Marker::ADD;
      marker.points.push_back(latest_cone_state.tip);
      marker.points.push_back(latest_rnw_state.cmd_setpoint);

      bool show_cmd = rnw_cmd_ontime() && latest_rnw_state.is_walking;

      if ( !show_cmd ) {
        marker.action = visualization_msgs::Marker::DELETE;
      }

      return marker;

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

      if ( latest_cone_state.is_point_contact ) {
        marker.action = visualization_msgs::Marker::ADD;
        auto arrow_tip = latest_cone_state.contact_point;
        arrow_tip.z += 0.5;
        marker.points.push_back(latest_cone_state.contact_point);
        marker.points.push_back(arrow_tip);
      } else {
        marker.action = visualization_msgs::Marker::DELETE;
      }

      return marker;

    }

    visualization_msgs::Marker gen_marker_moai() const {

      visualization_msgs::Marker marker;

      marker.id = id_moai;

      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = "package://rnw_ros/meshes/moai.dae";
      marker.mesh_use_embedded_materials = true;

      //marker.type = visualization_msgs::Marker::CYLINDER;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "moai";
      //marker.color = color;

      marker.pose.orientation = latest_cone_state.odom.pose.pose.orientation;
      marker.pose.position = latest_cone_state.disc_center;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;

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
      marker.color = color;
      marker.pose.orientation.w = 1;
      marker.scale.x = 0.01;
      marker.points.push_back(latest_cone_state.base);
      marker.points.push_back(latest_cone_state.tip);
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
      marker.color = color;

      marker.pose.orientation = latest_cone_state.odom.pose.pose.orientation;
      marker.pose.position = latest_cone_state.disc_center;
      marker.scale.x = latest_cone_state.radius * 2;
      marker.scale.y = latest_cone_state.radius * 2;
      marker.scale.z = 0.01;

      return marker;

    }

    visualization_msgs::Marker gen_contact_path(){
      if ( latest_cone_state.is_point_contact ) {
        marker_contact_path.points.push_back(latest_cone_state.contact_point);
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

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_cone_node");

  ros::NodeHandle nh("~");

  cone_visualizer_t cone_viz(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}