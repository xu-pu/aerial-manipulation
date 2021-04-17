#include "rnw_ros/traj_uitls.h"
#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/poly_traj.h"
#include "n3ctrl/N3CtrlState.h"

double odom2yaw( nav_msgs::Odometry const & odom ){
  Matrix3d R = uav_utils::from_quaternion_msg(odom.pose.pose.orientation).toRotationMatrix();
  return quat2eulers(Quaterniond(R)).z();
}

struct traj_server_t {

    ros::Publisher pub_pos_cmd;

    poly_traj_t poly_traj;

    quadrotor_msgs::PolynomialTrajectory latest_poly_traj;

    nav_msgs::Odometry cur_uav_odom;

    uint32_t traj_id = 1;

    double base_yaw = 0;

    double yaw_rate_deg = 1;

    double yaw_rate = 1;

    explicit traj_server_t( ros::NodeHandle & nh ){
      pub_pos_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd",10);
      yaw_rate_deg = get_param_default<double>(nh,"yaw_rate_deg",30.);
      yaw_rate = yaw_rate_deg * deg2rad;
    }

    void on_cleanup(){
      poly_traj = poly_traj_t();
    }

    void on_odom( OdometryConstPtr const & odom ){

      cur_uav_odom = *odom;

      ros::Time t = ros::Time::now();

      if ( poly_traj.available && t < poly_traj.final_time ) {

        // position command
        quadrotor_msgs::PositionCommand cmd;
        poly_traj.gen_pos_cmd(cmd,*odom,t,yaw_rate);
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id;
        pub_pos_cmd.publish(cmd);
      }
      else if ( poly_traj.available ) {
        // traj finished
        quadrotor_msgs::PositionCommand cmd;
        poly_traj.gen_pos_cmd(cmd,*odom,t,yaw_rate);
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_COMPLETED;
        cmd.trajectory_id = traj_id;
        pub_pos_cmd.publish(cmd);
        on_cleanup();
      }
      else {
        // idle
      }
    }

    void on_poly_traj( quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg ){
      on_cleanup();
      poly_traj = poly_traj_t(*msg);
      latest_poly_traj = *msg;
      base_yaw = odom2yaw(cur_uav_odom);
      traj_id = traj_id+1;
      ROS_INFO_STREAM("[Traj Server] Start new trajectory with id #" << traj_id);
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"traj2cmd_node");

  ros::NodeHandle nh("~");

  traj_server_t traj_server(nh);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "odom",
          10,
          &traj_server_t::on_odom,
          &traj_server,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_poly_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>(
          "traj",
          10,
          &traj_server_t::on_poly_traj,
          &traj_server,
          ros::TransportHints().tcpNoDelay()
  );

  ros::spin();

  ros::shutdown();

  return 0;

}