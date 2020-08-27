#include "rnw_ros/poly_traj.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

using Eigen::Vector3d;

struct cone_visualizer_t {

    double clear_after_n_sec = numeric_limits<double>::max();

    /**
     * record the receiving time using ros::Time::now() so it works in rosbag playback
     */
    ros::Time latest_time;

    nav_msgs::Odometry latest_odom;

    poly_traj_t poly_traj;

    bool init = false;

    ros::Publisher pub_marker_cone;

    explicit cone_visualizer_t( ros::NodeHandle & nh ) {
      pub_marker_cone = nh.advertise<visualization_msgs::Marker>("markers/cone", 1);
      clear_after_n_sec = get_param_default(nh,"clear_after_n_sec",numeric_limits<double>::max());
    }

    void on_odom( nav_msgs::OdometryConstPtr const & msg  ){

      latest_odom = *msg;
      latest_time = ros::Time::now();

      pub_marker_cone.publish(gen_marker_traj());

      init = true;

    }

    void on_spin( const ros::TimerEvent &event ){
      if ( !init ) { return; }
      if ( (ros::Time::now() - latest_time).toSec() > poly_traj.duration() + clear_after_n_sec ) {

      }
    }

    visualization_msgs::Marker gen_marker_traj(){

      constexpr int id = 0;
      constexpr double dt = 0.01;

      visualization_msgs::Marker marker;

      marker.id = id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "world";
      marker.pose.orientation.w = 1.00;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "test";
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

    void clear_traj_markers() const {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETEALL;
      pub_marker_cone.publish(marker);
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