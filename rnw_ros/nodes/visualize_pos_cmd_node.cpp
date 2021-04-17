//
// Created by sheep on 2021/4/17.
//

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <uav_utils/converters.h>

static constexpr int marker_id_acc = 1;
static constexpr int marker_id_vel = 2;
static constexpr int marker_id_pos = 3;

static constexpr double arrow_scale_x = 0.01;
static constexpr double arrow_scale_y = 0.03;
static constexpr double arrow_scale_z = 0.10;

static constexpr double length_g = 0.5;
static constexpr double vel_scale = 1;

visualization_msgs::MarkerArray gen_maker_clear(){
  visualization_msgs::MarkerArray rst;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  rst.markers.push_back(marker);
  return rst;
}

//visualization_msgs::Marker gen_maker_pos( quadrotor_msgs::PositionCommand const & msg ){}

visualization_msgs::Marker gen_maker_vel( quadrotor_msgs::PositionCommand const & msg ){

  visualization_msgs::Marker marker;

  marker.id = marker_id_vel;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.pose.orientation.w = 1.00;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.stamp = ros::Time::now();
  marker.ns = "viz_lift";
  marker.color.r = 255.0 / 255.0;
  marker.color.g = 20.0 / 255.0;
  marker.color.b = 147.0 / 255.0;
  marker.color.a = 1.0;
  marker.scale.x = arrow_scale_x;
  marker.scale.y = arrow_scale_y;
  marker.scale.z = arrow_scale_z;

  marker.points.clear();
  marker.points.push_back(msg.position);

  Eigen::Vector3d pos = uav_utils::from_point_msg(msg.position);
  Eigen::Vector3d vel = uav_utils::from_vector3_msg(msg.velocity);
  Eigen::Vector3d endpt = pos + vel_scale*vel;

  marker.points.emplace_back(uav_utils::to_point_msg(endpt));

  return marker;

}

visualization_msgs::Marker gen_maker_acc( quadrotor_msgs::PositionCommand const & msg ){

  Eigen::Vector3d g = { 0, 0, -9.8 };

  visualization_msgs::Marker marker;

  marker.id = marker_id_acc;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.pose.orientation.w = 1.00;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.stamp = ros::Time::now();
  marker.ns = "viz_lift";
  marker.color.r = 255.0 / 255.0;
  marker.color.g = 20.0 / 255.0;
  marker.color.b = 147.0 / 255.0;
  marker.color.a = 1.0;
  marker.scale.x = arrow_scale_x;
  marker.scale.y = arrow_scale_y;
  marker.scale.z = arrow_scale_z;

  marker.points.clear();
  marker.points.push_back(msg.position);

  Eigen::Vector3d X = uav_utils::from_point_msg(msg.position);
  Eigen::Vector3d acc = uav_utils::from_vector3_msg(msg.acceleration);
  Eigen::Vector3d lift = (acc - g)/std::abs(g.z())*length_g;
  X += lift;
  marker.points.push_back(uav_utils::to_point_msg(X));

  return marker;

}

visualization_msgs::MarkerArray gen_maker_pos_cmd( quadrotor_msgs::PositionCommand const & msg ){
  visualization_msgs::MarkerArray marker;
  //marker.markers.emplace_back(gen_maker_pos(msg));
  marker.markers.emplace_back(gen_maker_vel(msg));
  marker.markers.emplace_back(gen_maker_acc(msg));
  return marker;
}

struct pos_cmd_viz_t {

    ros::NodeHandle & nh;

    ros::Subscriber sub_pos_cmd;

    ros::Publisher pub_markers;

    ros::Timer timer;

    quadrotor_msgs::PositionCommand latest_pos_cmd;

    explicit pos_cmd_viz_t( ros::NodeHandle & _nh ): nh(_nh){

      pub_markers = nh.advertise<visualization_msgs::MarkerArray>("makers",10);

      sub_pos_cmd = nh.subscribe<quadrotor_msgs::PositionCommand>(
              "position_cmd",
              10,
              &pos_cmd_viz_t::on_pos_cmd,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      timer = nh.createTimer(ros::Rate(30),&pos_cmd_viz_t::on_timer,this);

    }

    void on_pos_cmd( quadrotor_msgs::PositionCommandConstPtr const & msg ){
      latest_pos_cmd = *msg;
    }

    void on_timer( ros::TimerEvent const & event ) const {
      visualization_msgs::MarkerArray marker = gen_maker_clear();
      if ( (ros::Time::now() - latest_pos_cmd.header.stamp).toSec() < 1 ) {
        marker = gen_maker_pos_cmd(latest_pos_cmd);
      }
      pub_markers.publish(marker);
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_pos_cmd_node");

  ros::NodeHandle nh("~");

  pos_cmd_viz_t pos_cmd_viz(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}