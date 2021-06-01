#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <rnw_ros/ros_utils.h>


Eigen::Vector3d gps2pos( sensor_msgs::NavSatFix const & msg ){

  static bool init = false;
  static Eigen::Vector3d gps_origin;

  constexpr double EARTH_RADIUS = 6378137.0;

  ROS_ASSERT(msg.header.frame_id == "NED");

  Eigen::Vector3d position_abs(
          msg.latitude * deg2rad * EARTH_RADIUS,
          -msg.longitude * deg2rad * EARTH_RADIUS,
          msg.altitude
  );

  if (!init) {
    init = true;
    gps_origin = position_abs;
  }

  return position_abs - gps_origin;

}

struct sensor_sim_t {

    ros::Subscriber sub_imu;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_gps;

    ros::Publisher pub_imu;
    ros::Publisher pub_odom;

    //std::deque<sensor_msgs::Imu>

    sensor_msgs::Imu latest_imu;

    sensor_msgs::NavSatFix latest_gps;

    geometry_msgs::Vector3 latest_acc;

    //gps_estimator_t gps;

    sensor_sim_t(){

      ros::NodeHandle nh("~");

      sub_imu = nh.subscribe<sensor_msgs::Imu>(
              "imu_in",
              1000,
              &sensor_sim_t::on_imu,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_gps = nh.subscribe<sensor_msgs::NavSatFix>(
              "gps",
              1000,
              &sensor_sim_t::on_gps,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      pub_odom = nh.advertise<nav_msgs::Odometry>("odom",100);

      pub_imu = nh.advertise<sensor_msgs::Imu>("imu_out",100);

    }

    /**
     * The HIL simulator in N3 has wrong linear acceleration
     * @param msg
     */
    void on_imu( sensor_msgs::ImuConstPtr const & msg ){
      latest_imu = *msg;
      latest_imu.linear_acceleration = latest_acc;
      pub_imu.publish(latest_imu);
    }

    void on_gps( sensor_msgs::NavSatFixConstPtr const & msg ){
      latest_gps = *msg;
      Vector3d pos = gps2pos(*msg);
      nav_msgs::Odometry odom;
      odom.header.stamp = msg->header.stamp;
      odom.header.frame_id = "world";
      odom.pose.pose.orientation = latest_imu.orientation;
      odom.pose.pose.position = uav_utils::to_point_msg(pos);
      pub_odom.publish(odom);
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_simulated_sensors");

  sensor_sim_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}