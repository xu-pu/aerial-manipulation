//
// Created by sheep on 2021/4/17.
//

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
static constexpr double length_1kg = 0.5;
static constexpr double g = 9.8;

struct disturbance_viz_t {

    ros::NodeHandle & nh;

    ros::Subscriber sub_disturbance;

    ros::Subscriber sub_odom;

    ros::Publisher pub_markers;

    ros::Timer timer;

    nav_msgs::Odometry latest_odom;

    geometry_msgs::Vector3Stamped latest_disturbance;

    explicit disturbance_viz_t( ros::NodeHandle & _nh ): nh(_nh){

      pub_markers = nh.advertise<visualization_msgs::Marker>("marker",10);

      sub_odom = nh.subscribe<nav_msgs::Odometry>(
              "odom",
              10,
              &disturbance_viz_t::on_odom,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_disturbance = nh.subscribe<geometry_msgs::Vector3Stamped>(
              "disturbance",
              10,
              &disturbance_viz_t::on_disturbance,
              this,
              ros::TransportHints().tcpNoDelay()
      );

    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg ){
      latest_odom = *msg;
      pub_markers.publish(gen_maker());
    }

    void on_disturbance( geometry_msgs::Vector3StampedConstPtr const & msg ){
      latest_disturbance = *msg;
    }

    visualization_msgs::MarkerArray gen_maker_clear(){
      visualization_msgs::MarkerArray rst;
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      rst.markers.push_back(marker);
      return rst;
    }

    visualization_msgs::Marker gen_maker(){

      visualization_msgs::Marker marker;

      marker.id = marker_id_acc;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.stamp = ros::Time::now();
      marker.ns = "viz_disturbance";
      marker.color.r = 255.0 / 255.0;
      marker.color.g = 20.0 / 255.0;
      marker.color.b = 147.0 / 255.0;
      marker.color.a = 1.0;
      marker.scale.x = arrow_scale_x;
      marker.scale.y = arrow_scale_y;
      marker.scale.z = arrow_scale_z;

      marker.points.clear();
      marker.points.push_back(latest_odom.pose.pose.position);

      Eigen::Vector3d pos = uav_utils::from_point_msg(latest_odom.pose.pose.position);
      Eigen::Vector3d dis = uav_utils::from_vector3_msg(latest_disturbance.vector);
      Eigen::Vector3d vec = dis * length_1kg / g;
      Eigen::Vector3d endpoint = pos + vec;
      marker.points.push_back(uav_utils::to_point_msg(endpoint));

      return marker;

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_pos_cmd_node");

  ros::NodeHandle nh("~");

  disturbance_viz_t node(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}