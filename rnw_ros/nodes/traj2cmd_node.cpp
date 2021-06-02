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

    ros::Subscriber sub_odom;

    ros::Subscriber sub_poly_traj;

    ros::Subscriber sub_abort;

    poly_traj_t poly_traj;

    quadrotor_msgs::PolynomialTrajectory latest_poly_traj;

    nav_msgs::Odometry cur_uav_odom;

    uint32_t traj_id = 1;

    double base_yaw = 0;

    double yaw_rate_deg = 1;

    double yaw_rate = 1;

    bool latch = true;

    explicit traj_server_t( ros::NodeHandle & nh ){

      latch = get_param_default<bool>(nh,"latch",true);
      yaw_rate_deg = get_param_default<double>(nh,"yaw_rate_deg",30.);
      yaw_rate = yaw_rate_deg * deg2rad;

      pub_pos_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd",10);

      sub_odom = nh.subscribe<nav_msgs::Odometry>(
              "odom",
              10,
              &traj_server_t::on_odom,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_poly_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>(
              "traj",
              10,
              &traj_server_t::on_poly_traj,
              this,
              ros::TransportHints().tcpNoDelay()
      );

      sub_abort = nh.subscribe<std_msgs::Header>(
              "/abort",
              100,
              &traj_server_t::on_abort,
              this,
              ros::TransportHints().tcpNoDelay()
      );

    }

    void on_cleanup(){
      poly_traj = poly_traj_t();
    }

    void on_abort( std_msgs::HeaderConstPtr const & msg ){
      ROS_ERROR_STREAM("[traj2cmd] trajectory aborted!");
      on_cleanup();
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
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id;
        pub_pos_cmd.publish(cmd);
        if ( !latch ) {
          on_cleanup();
        }
      }
      else {
        // idle
      }
    }

    void on_poly_traj( quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg ){
      on_cleanup();
      if ( msg->action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT ) {
        ROS_INFO_STREAM("[traj2cmd] ACTION_ABORT");
      }
      else if ( msg->action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD ) {
        poly_traj = poly_traj_t(*msg);
        latest_poly_traj = *msg;
        base_yaw = odom2yaw(cur_uav_odom);
        traj_id = traj_id+1;
        ROS_INFO_STREAM("[traj2cmd] ACTION_ADD");
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"traj2cmd_node");

  ros::NodeHandle nh("~");

  traj_server_t traj_server(nh);

  ros::spin();

  ros::shutdown();

  return 0;

}