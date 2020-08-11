#include "rnw_ros/poly_traj.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

using Eigen::Vector3d;

struct traj_visualizer_t {

    double clear_after_n_sec = 1;

    double lift_dt = 0.15;

    double acc_dt = 0.15;

    quadrotor_msgs::PolynomialTrajectory latest_msg;

    poly_traj_t poly_traj;

    bool init = false;

    ros::Publisher pub_marker_traj;
    ros::Publisher pub_marker_acc;
    ros::Publisher pub_marker_lift;

    explicit traj_visualizer_t( ros::NodeHandle & nh ) {
      pub_marker_traj = nh.advertise<visualization_msgs::Marker>("markers/traj", 1);
      pub_marker_acc = nh.advertise<visualization_msgs::MarkerArray>("markers/acc", 1);
      pub_marker_lift = nh.advertise<visualization_msgs::MarkerArray>("markers/lift", 1);

      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",1);
      lift_dt = get_param_default(nh,"lift_dt",0.15);
      acc_dt = get_param_default(nh,"acc_dt",0.15);

    }

    void on_traj(  quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg  ){
      latest_msg = *msg;
      poly_traj = poly_traj_t(latest_msg);

      //clear_acc_markers();
      clear_lift_markers();
      //pub_marker_acc.publish(gen_marker_acc());
      pub_marker_lift.publish(gen_marker_lift());
      pub_marker_traj.publish(gen_marker_traj());

      init = true;
    }

    void on_spin( const ros::TimerEvent &event ){
      if ( !init ) { return; }
      if ( (ros::Time::now() - latest_msg.header.stamp).toSec() > poly_traj.duration() + clear_after_n_sec ) {
        clear_markers();
      }
    }

    visualization_msgs::Marker gen_marker_traj(){

      constexpr int id = 0;
      constexpr double dt = 0.01;

      visualization_msgs::Marker marker;

      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "test";
      marker.scale.x = 0.15;
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
      marker.ns = "viz_traj";
      marker.color.r = 255.0 / 255.0;
      marker.color.g = 20.0 / 255.0;
      marker.color.b = 147.0 / 255.0;
      marker.color.a = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.15;
      marker.scale.z = 0.30;

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
      marker.scale.x = 0.05;
      marker.scale.y = 0.15;
      marker.scale.z = 0.30;

      for (double t = 0; t < poly_traj.duration(); t += lift_dt){
        marker.id += 3;
        marker.points.clear();
        geometry_msgs::Point point;
        Vector3d X = poly_traj.eval_pos(t);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        marker.points.push_back(point);
        Vector3d lift = poly_traj.eval_acc(t) - g;
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
      clear_lift_markers();
      clear_acc_markers();
    }

};

void on_spin(){}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_traj_node");

  ros::NodeHandle nh("~");

  traj_visualizer_t traj_viz(nh);

  constexpr size_t spin_hz = 10;

  auto timer = nh.createTimer( ros::Duration( 1.0 / spin_hz ), &traj_visualizer_t::on_spin, &traj_viz );

  ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>("poly_traj", 100, &traj_visualizer_t::on_traj, &traj_viz );

  ros::spin();

  traj_viz.clear_markers();

  ros::shutdown();

  return 0;

}