//
// Created by sheep on 2021/4/17.
//

#include "rnw_ros/cone_interface.h"

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Dense>

#include <uav_utils/converters.h>

static constexpr int marker_id_acc = 1;

static constexpr double arrow_scale_x = 0.01;
static constexpr double arrow_scale_y = 0.03;
static constexpr double arrow_scale_z = 0.10;
static constexpr double length_1kg = 1;
static constexpr double g = 9.8;
string marker_ns("viz_rl_cp_vel");

struct rl_viz_t {

    ros::NodeHandle & nh;

    ros::Subscriber sub_cp_vel;

    ros::Publisher pub_markers;

    ros::Timer timer;

    geometry_msgs::Vector3Stamped latest_cp_vel;

    cone_interface_t cone;

    ros::Timer update_loop;

    explicit rl_viz_t( ros::NodeHandle & _nh ): nh(_nh){

      pub_markers = nh.advertise<visualization_msgs::Marker>("marker",10);

      sub_cp_vel = nh.subscribe<geometry_msgs::Vector3Stamped>(
              "/rl_agent/action_cp_vel",
              10,
              &rl_viz_t::on_cp_vel,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      update_loop = nh.createTimer(
              ros::Rate(30),
              &rl_viz_t::on_update_loop,
              this
      );

    }

    void on_update_loop( ros::TimerEvent const & e ) const {
      if ( cone.odom_in_time() && latency(latest_cp_vel) < 0.5 ) {
        pub_markers.publish(gen_maker());
      }
      else {
        pub_markers.publish(gen_maker_clear());
      }
    }

    void on_cp_vel( geometry_msgs::Vector3StampedConstPtr const & msg ){
      latest_cp_vel = *msg;
    }

    static visualization_msgs::Marker gen_maker_clear() {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      return marker;
    }

    visualization_msgs::Marker gen_maker() const {

      visualization_msgs::Marker marker;

      marker.id = marker_id_acc;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.stamp = ros::Time::now();
      marker.ns = marker_ns;
      marker.color.r = 255.0 / 255.0;
      marker.color.g = 20.0 / 255.0;
      marker.color.b = 147.0 / 255.0;
      marker.color.a = 1.0;
      marker.scale.x = arrow_scale_x;
      marker.scale.y = arrow_scale_y;
      marker.scale.z = arrow_scale_z;

      marker.points.clear();
      marker.points.push_back(cone.latest_cone_state.tip);

      Eigen::Vector3d pos = uav_utils::from_point_msg(cone.latest_cone_state.tip);
      Eigen::Vector3d dis = uav_utils::from_vector3_msg(latest_cp_vel.vector);
      Eigen::Vector3d vec = dis * length_1kg;
      Eigen::Vector3d endpoint = pos + vec;
      marker.points.push_back(uav_utils::to_point_msg(endpoint));

      return marker;

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_rl_node");

  ros::NodeHandle nh("~");

  rl_viz_t node(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}