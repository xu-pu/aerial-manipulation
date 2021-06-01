#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <rnw_ros/ros_utils.h>

struct vel_acc_estimator_t {

    Vector3d vel;

    Vector3d acc;

    void update( Vector3d const & p, ros::Time const t ){

      p2 = p1;
      p1 = p0;
      p0 = p;

      t2 = t1;
      t1 = t0;
      t0 = t;

      if ( counter > 3 ) {
        vel = (p0-p1)/(t0-t1).toSec();
        Vector3d vel_pre = (p1-p2)/(t1-t2).toSec();
        acc = (vel - vel_pre)/(t0-t1).toSec();
      }
      else {
        vel = Vector3d::Zero();
        acc = Vector3d::Zero();
      }

      counter++;

    }

private:

    Vector3d p0;
    Vector3d p1;
    Vector3d p2;

    ros::Time t0;
    ros::Time t1;
    ros::Time t2;

    size_t counter = 0;

};

struct odom_with_latency_t {

    ros::Publisher pub_odom;

    std::deque<nav_msgs::Odometry> timeline;

    double latency_sec = 0;

    odom_with_latency_t(){
      ros::NodeHandle nh("~");
      pub_odom = nh.advertise<nav_msgs::Odometry>("odom",100);
      latency_sec = get_param_default<double>(nh,"odom_latency_ms",0.) / 1000.;
    }

    void update( nav_msgs::Odometry const & odom ){

      if ( !timeline.empty() && odom.header.stamp <= timeline.back().header.stamp ) {
        ROS_ERROR_STREAM("[sensor sim] out of order odom");
      }
      else {
        timeline.emplace_back(odom);
      }

      std::deque<nav_msgs::Odometry> popped;

      while ( !timeline.empty() ) {
        if ( (ros::Time::now() - timeline.back().header.stamp).toSec() > latency_sec ) {
          popped.emplace_back(timeline.back());
          timeline.pop_front();
        }
      }

      if ( !popped.empty() ) {
        pub_odom.publish(popped.back());
      }

    }

};

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

    sensor_msgs::Imu latest_imu;

    sensor_msgs::NavSatFix latest_gps;

    vel_acc_estimator_t vel_acc;

    odom_with_latency_t odom_with_latency;

    Vector3d g = Vector3d(0,0,9.81);

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

      pub_imu = nh.advertise<sensor_msgs::Imu>("imu_out",100);

    }

    /**
     * The HIL simulator in N3 has wrong linear acceleration
     * @param msg
     */
    void on_imu( sensor_msgs::ImuConstPtr const & msg ){
      latest_imu = *msg;
      latest_imu.linear_acceleration = uav_utils::to_vector3_msg(
              uav_utils::from_quaternion_msg(latest_imu.orientation).inverse() * (vel_acc.acc+g)
      );
      pub_imu.publish(latest_imu);
    }

    void on_gps( sensor_msgs::NavSatFixConstPtr const & msg ){

      latest_gps = *msg;
      Vector3d pos = gps2pos(*msg);
      vel_acc.update(pos,latest_gps.header.stamp);

      nav_msgs::Odometry odom;
      odom.header.stamp = msg->header.stamp;
      odom.header.frame_id = "world";
      odom.pose.pose.orientation = latest_imu.orientation;
      odom.pose.pose.position = uav_utils::to_point_msg(pos);
      odom.twist.twist.angular = latest_imu.angular_velocity;
      odom.twist.twist.linear = uav_utils::to_vector3_msg(vel_acc.vel);

      odom_with_latency.update(odom);

    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"pub_simulated_sensors");

  sensor_sim_t node;

  ros::spin();

  ros::shutdown();

  return 0;

}