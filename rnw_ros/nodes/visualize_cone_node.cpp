#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_planner.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rnw_msgs/ConeState.h>
#include <rnw_msgs/RockingCmd.h>
#include <rnw_msgs/GripState.h>

using namespace std;

struct cone_visualizer_t {

    rnw_config_t rnw_cfg;

    double clear_after_n_sec = numeric_limits<double>::max();

    ros::Time latest_time;

    ros::Publisher pub_marker_cone;

    bool cone_state_init = false;

    rnw_msgs::ConeState latest_cone_state;

    bool rocking_cmd_init = false;

    rnw_msgs::RockingCmd latest_rocking_cmd;

    bool grip_state_init = false;

    rnw_msgs::GripState latest_grip_state;

    static constexpr int id_base = 0;
    static constexpr int id_shaft = 1;
    static constexpr int id_contact_path = 2;
    static constexpr int id_contact_normal = 3;
    static constexpr int id_rocking_cmd = 4;
    static constexpr int id_grip = 5;

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
      pub_marker_cone = nh.advertise<visualization_msgs::MarkerArray>("markers/cone", 1);
      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",numeric_limits<double>::max());
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
      latest_time = ros::Time::now();
      latest_cone_state = *msg;
      pub_marker_cone.publish(gen_markers());
      cone_state_init = true;

      if ( cone_is_qstatic(latest_cone_state,rnw_cfg) ) {
        color = color_qstatic;
      }
      else {
        color = color_default;
      }

    }

    void on_rocking_cmd( rnw_msgs::RockingCmdConstPtr const & msg ){
      latest_rocking_cmd = *msg;
      rocking_cmd_init = true;
    }

    void on_grip_state( rnw_msgs::GripStateConstPtr const & msg ){
      latest_grip_state = *msg;
      grip_state_init = true;
    }

    visualization_msgs::MarkerArray gen_markers(){
      visualization_msgs::MarkerArray marker_arr;
      marker_arr.markers.push_back(gen_marker_base());
      marker_arr.markers.push_back(gen_marker_shaft());
      marker_arr.markers.push_back(gen_contact_path());
      marker_arr.markers.push_back(gen_marker_contact_normal());
      marker_arr.markers.push_back(gen_marker_rocking_cmd());
      marker_arr.markers.push_back(gen_maker_grip());
      return marker_arr;
    }

    visualization_msgs::Marker gen_marker_rocking_cmd() const {

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
      marker.points.push_back(latest_rocking_cmd.grip_state.grip_point);
      marker.points.push_back(latest_rocking_cmd.setpoint_apex);

      switch (latest_rocking_cmd.cmd_type) {
        case rnw_cmd_t::cmd_rocking:
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 0;
          break;
        case rnw_cmd_t::cmd_adjust_grip:
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;
          break;
        case rnw_cmd_t::cmd_adjust_nutation:
          marker.color.r = 0;
          marker.color.g = 0;
          marker.color.b = 1;
          break;
        default:
          ROS_ERROR("[rnw_visual] invalid rnw cmd type!");
      }

      bool dont_show_cmd = sec_since_msg(latest_rocking_cmd) > 0.5 || latest_rocking_cmd.fsm == rnw_cmd_t::fsm_idle;

      if ( dont_show_cmd ) {
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

    visualization_msgs::Marker gen_maker_grip() const {

      visualization_msgs::Marker marker;

      marker.id = id_grip;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.ns = ns;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 0;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 1.00;
      marker.pose.orientation.w = 1;

      bool show_grip = grip_state_init
                       && latest_grip_state.grip_valid
                       && sec_since_msg(latest_grip_state) < 0.5;

      if ( show_grip ) {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = latest_grip_state.grip_point;
      }
      else {
        marker.action = visualization_msgs::Marker::DELETE;
      }

      return marker;

    }

    void clear_markers() const {
      visualization_msgs::MarkerArray accMarkers;
      visualization_msgs::Marker accMarker;
      accMarker.action = visualization_msgs::Marker::DELETEALL;
      accMarkers.markers.push_back(accMarker);
      pub_marker_cone.publish(accMarkers);
    }

    void on_spin( const ros::TimerEvent &event ) const {
      if ( !cone_state_init ) { return; }
      if ( sec_since(latest_time) > clear_after_n_sec ) {
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

  ros::Subscriber sub_traj = nh.subscribe<rnw_msgs::ConeState>(
          "/rnw/cone_state",
          100,
          &cone_visualizer_t::on_cone_state,
          &cone_viz,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_rocking_cmd = nh.subscribe<rnw_msgs::RockingCmd>(
          "/rnw/rocking_cmd",
          100,
          &cone_visualizer_t::on_rocking_cmd,
          &cone_viz,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_grip_state = nh.subscribe<rnw_msgs::GripState>(
          "/rnw/grip_state",
          100,
          &cone_visualizer_t::on_grip_state,
          &cone_viz,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}