#include "rnw_ros/poly_traj.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

using Eigen::Vector3d;

struct traj_visualizer_t {

    double clear_after_n_sec = numeric_limits<double>::max();

    double lift_dt = 0.15;

    double acc_dt = 0.15;

    double length_g = 1;

    static constexpr double arrow_scale_x = 0.01;

    static constexpr double arrow_scale_y = 0.03;

    static constexpr double arrow_scale_z = 0.10;

    static constexpr size_t id_traj = 0;
    static constexpr size_t id_acc = 1;
    static constexpr size_t id_lift = 2;
    static constexpr size_t id_setpoint = 3;

    string ns = "visualize_traj_node";

    /**
     * record the receiving time using ros::Time::now() so it works in rosbag playback
     */
    ros::Time latest_time;

    quadrotor_msgs::PolynomialTrajectory latest_msg;

    poly_traj_t poly_traj;

    Vector3d setpoint_pos;

    Vector3d setpoint_acc;

    bool init = false;

    ros::Publisher pub_marker_traj;
    ros::Publisher pub_marker_setpoint;
    ros::Publisher pub_marker_acc;
    ros::Publisher pub_marker_lift;

    ros::Timer timer;
    ros::Subscriber sub_traj;
    ros::Subscriber sub_abort;

    explicit traj_visualizer_t( ros::NodeHandle & nh ) {
      pub_marker_traj = nh.advertise<visualization_msgs::Marker>("traj", 1);
      pub_marker_setpoint = nh.advertise<visualization_msgs::Marker>("setpoint", 1);
      pub_marker_acc = nh.advertise<visualization_msgs::MarkerArray>("acc", 1);
      pub_marker_lift = nh.advertise<visualization_msgs::MarkerArray>("lift", 1);

      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",numeric_limits<double>::max());
      lift_dt = get_param_default(nh,"lift_dt",0.15);
      acc_dt = get_param_default(nh,"acc_dt",0.15);
      length_g = get_param_default<double>(nh,"length_g",1.);

      timer = nh.createTimer( ros::Rate(30), &traj_visualizer_t::on_spin, this );

      sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>(
              "poly_traj",
              100,
              &traj_visualizer_t::on_traj,
              this
      );

      sub_traj = nh.subscribe<std_msgs::Header>(
              "/abort",
              100,
              &traj_visualizer_t::on_abort,
              this
      );

    }

    void on_abort( std_msgs::HeaderConstPtr const & msg ){
      init = false;
      clear_markers();
    }

    void on_traj(  quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg  ){
      if ( msg->action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT ) {
        init = false;
        clear_markers();
        ROS_INFO_STREAM("[viz_traj] ACTION_ABORT");
      }
      else if ( msg->action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD ) {
        latest_msg = *msg;
        poly_traj = poly_traj_t(latest_msg);
        latest_time = ros::Time::now();
        clear_acc_markers();
        clear_lift_markers();
        pub_marker_acc.publish(gen_marker_acc());
        pub_marker_lift.publish(gen_marker_lift());
        pub_marker_traj.publish(gen_marker_traj());
        init = true;
        ROS_INFO_STREAM("[viz_traj] ACTION_ADD");
      }
    }

    void on_spin( const ros::TimerEvent &event ){
      if ( !init ) { return; }
      else if ( (ros::Time::now() - latest_time).toSec() < poly_traj.duration() ) {
        // traj in progress
        setpoint_pos = poly_traj.eval_pos(ros::Time::now());
        setpoint_acc = poly_traj.eval_acc(ros::Time::now());
        pub_marker_setpoint.publish(gen_marker_setpoint());
      }
      else if ( (ros::Time::now() - latest_time).toSec() > poly_traj.duration() + clear_after_n_sec ) {
        // traj finished
        clear_markers();
      }
    }

    visualization_msgs::Marker gen_marker_traj(){

      constexpr double dt = 0.01;

      visualization_msgs::Marker marker;

      marker.id = id_traj;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = ns;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;
      marker.color.r = 0.00;
      marker.color.g = 1.00;
      marker.color.b = 0.00;
      marker.color.a = 1.00;

      Vector3d lastX = poly_traj.eval_pos(0);
      for (double t = dt; t < poly_traj.poly_duration(); t += dt){
        geometry_msgs::Point point;
        Vector3d X = poly_traj.eval_pos(t);
        point.x = lastX(0);
        point.y = lastX(1);
        point.z = lastX(2);
        marker.points.push_back(point);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        marker.points.push_back(point);
        lastX = X;
      }

      return marker;

    }

    visualization_msgs::MarkerArray gen_marker_acc(){

      constexpr int id = 0;

      visualization_msgs::Marker marker;
      visualization_msgs::MarkerArray accMarkers;

      marker.id = id;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.stamp = ros::Time::now();
      marker.ns = "viz_acc";
      marker.color.r = 255.0 / 255.0;
      marker.color.g = 20.0 / 255.0;
      marker.color.b = 0;
      marker.color.a = 1.0;
      marker.scale.x = arrow_scale_x;
      marker.scale.y = arrow_scale_y;
      marker.scale.z = arrow_scale_z;

      for (double t = 0; t < poly_traj.duration(); t += acc_dt){
        marker.id += 3;
        marker.points.clear();
        geometry_msgs::Point point;
        Vector3d X = poly_traj.eval_pos(t);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        marker.points.push_back(point);
        X += poly_traj.eval_acc(t);
        //X += Vector3d {0,0,1};
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        marker.points.push_back(point);
        accMarkers.markers.push_back(marker);
      }

      return accMarkers;

    }

    visualization_msgs::MarkerArray gen_marker_lift(){

      constexpr int id = 0;

      Vector3d g = { 0, 0, -9.8 };

      visualization_msgs::Marker marker;
      visualization_msgs::MarkerArray accMarkers;

      marker.id = id;
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

      for (double t = 0; t < poly_traj.duration(); t += lift_dt){
        marker.id += 3;
        marker.points.clear();
        geometry_msgs::Point point;
        Vector3d X = poly_traj.eval_pos(t);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        marker.points.push_back(point);
        Vector3d lift = (poly_traj.eval_acc(t) - g)/abs(g.z())*length_g;
        X += lift;
        //X += Vector3d {0,0,1};
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        marker.points.push_back(point);
        accMarkers.markers.push_back(marker);
      }

      return accMarkers;

    }

    visualization_msgs::Marker gen_marker_setpoint(){

      Vector3d g = { 0, 0, -9.8 };

      visualization_msgs::Marker marker;

      marker.id = id_setpoint;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.color.r = 255.0 / 255.0;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1.0;
      marker.scale.x = arrow_scale_x;
      marker.scale.y = arrow_scale_y;
      marker.scale.z = arrow_scale_z;

      marker.id += 3;
      marker.points.clear();
      geometry_msgs::Point point;
      Vector3d X = setpoint_pos;
      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      marker.points.push_back(point);
      Vector3d lift = (setpoint_acc - g)/abs(g.z())*length_g;
      X += lift;
      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      marker.points.push_back(point);

      return marker;

    }

    void clear_traj_markers() const {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      pub_marker_traj.publish(marker);
    }

    void clear_setpoint_markers() const {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      pub_marker_setpoint.publish(marker);
    }

    void clear_acc_markers() const {
      visualization_msgs::MarkerArray accMarkers;
      visualization_msgs::Marker accMarker;
      accMarker.action = visualization_msgs::Marker::DELETEALL;
      accMarkers.markers.push_back(accMarker);
      pub_marker_acc.publish(accMarkers);
    }

    void clear_lift_markers() const {
      visualization_msgs::MarkerArray accMarkers;
      visualization_msgs::Marker accMarker;
      accMarker.action = visualization_msgs::Marker::DELETEALL;
      accMarkers.markers.push_back(accMarker);
      pub_marker_lift.publish(accMarkers);
    }

    void clear_markers() const {
      clear_traj_markers();
      clear_lift_markers();
      clear_acc_markers();
      clear_setpoint_markers();
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_traj_node");

  ros::NodeHandle nh("~");

  traj_visualizer_t traj_viz(nh);

  auto timer = nh.createTimer( ros::Rate(30), &traj_visualizer_t::on_spin, &traj_viz );

  ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>("poly_traj", 100, &traj_visualizer_t::on_traj, &traj_viz );

  ros::spin();

  traj_viz.clear_markers();

  ros::shutdown();

  return 0;

}