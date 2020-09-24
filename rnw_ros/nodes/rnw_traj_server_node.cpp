#include "rnw_ros/traj_uitls.h"
#include "rnw_ros/pose_utils.h"
#include "rnw_ros/rnw_utils.h"
#include "rnw_ros/poly_traj.h"
#include "n3ctrl/N3CtrlState.h"

ros::Publisher pub_path_setpoint;
ros::Publisher pub_path_plant;
ros::Publisher pub_pos_cmd;

PolynomialTraj minsnap( vector<Vector3d> const & waypoints, double interval ){
  MatrixXd POS = toXd(waypoints);
  VectorXd TIMES = gen_time_intervals(interval,waypoints);
  return minSnapTraj(POS,Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),TIMES);
}

/**
 * N waypoints, N-1 time intervals.
 * i-th interval is time between i-th and (i+1)-th waypoints
 * @param waypoints
 * @param intervals
 * @return
 */
PolynomialTraj minsnap( vector<Vector3d> const & waypoints, vector<double> intervals ){
  MatrixXd POS = toXd(waypoints);
  VectorXd TIMES = gen_time_intervals(intervals,waypoints);
  return minSnapTraj(POS,Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),TIMES);
}

PoseStamped eigen2pathpoint( Vector3d const & T ){
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose.position = uav_utils::to_point_msg(T);
  return pose;
}

double odom2yaw( nav_msgs::Odometry const & odom ){
  Matrix3d R = uav_utils::from_quaternion_msg(odom.pose.pose.orientation).toRotationMatrix();
  return quat2eulers(Quaterniond(R)).z();
}

struct traj_server_t {

    poly_traj_t poly_traj;

    quadrotor_msgs::PolynomialTrajectory latest_poly_traj;

    n3ctrl::N3CtrlState latest_n3ctrl_state;

    nav_msgs::Odometry cur_uav_odom;

    geometry_msgs::PoseStamped trigger_pose;

    uint32_t traj_id = 1;

    double base_yaw = 0;

    double yaw_rate_deg = 1;

    double yaw_rate = 1;

    nav_msgs::Path path_setpoint;

    nav_msgs::Path path_plant;

    explicit traj_server_t( ros::NodeHandle & nh ){
      path_setpoint.header.frame_id = "world";
      path_plant.header.frame_id = "world";
      yaw_rate_deg = get_param_default<double>(nh,"yaw_rate_deg",30.);
      yaw_rate = yaw_rate_deg * deg2rad;
    }

    void on_completion(){
      traj_id++;
    }

    void on_trigger( geometry_msgs::PoseStampedConstPtr const & msg ){
      on_cleanup();
      trigger_pose = *msg;
      traj_id = latest_n3ctrl_state.last_traj_id+1;
      ROS_INFO_STREAM("[Traj Server] Triggered, last_traj_id: #" << latest_n3ctrl_state.last_traj_id);
    }

    void on_cleanup(){
      poly_traj = poly_traj_t();
      path_setpoint.poses.clear();
      path_plant.poses.clear();
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

        //path
        path_setpoint.poses.emplace_back(eigen2pathpoint({cmd.position.x,cmd.position.y,cmd.position.z}));
        path_plant.poses.emplace_back(eigen2pathpoint(odom2T(odom)));
        pub_path_setpoint.publish(path_setpoint);
        pub_path_plant.publish(path_plant);

      }
      else if ( poly_traj.available ) {
        // traj finished
        quadrotor_msgs::PositionCommand cmd;
        poly_traj.gen_pos_cmd(cmd,*odom,t,yaw_rate);
        cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_COMPLETED;
        cmd.trajectory_id = traj_id;
        pub_pos_cmd.publish(cmd);
        on_cleanup();
        on_completion();
      }
      else {
        // idle
      }
    }

    void on_poly_traj( quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg ){

      if ( latest_n3ctrl_state.state == n3ctrl::N3CtrlState::STATE_CMD_HOVER ||
           latest_n3ctrl_state.state == n3ctrl::N3CtrlState::STATE_CMD_CTRL )
      {
        on_cleanup();
        poly_traj = poly_traj_t(*msg);
        latest_poly_traj = *msg;
        base_yaw = odom2yaw(cur_uav_odom);
        traj_id = latest_n3ctrl_state.last_traj_id+1;
        ROS_INFO_STREAM("[Traj Server] Start new trajectory with id #" << traj_id);
      } else {
        ROS_WARN_STREAM("[Traj Server] Do not accept trajectory at current state of n3ctrl!");
      }

    }

    void on_n3ctrl_state( n3ctrl::N3CtrlStateConstPtr const & msg ){
      latest_n3ctrl_state = *msg;
      // when accident happens in n3ctrl, the old trajectory is aborted,
      // and last_traj_id will increase
      if ( poly_traj.available && latest_n3ctrl_state.last_traj_id >= traj_id ) {
        ROS_WARN_STREAM("[Traj Server] Traj #" << traj_id << " aborted by n3ctrl!");
        on_cleanup();
      }
    }

};

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_traj_server_node");

  ros::NodeHandle nh("~");

  traj_server_t traj_server(nh);

  pub_path_setpoint = nh.advertise<nav_msgs::Path>("/traj/setpoint",10);
  pub_path_plant = nh.advertise<nav_msgs::Path>("/traj/plant",10);
  pub_pos_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",10);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(
          "/odom/uav",
          10,
          &traj_server_t::on_odom,
          &traj_server,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_poly_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>(
          "/rnw/poly_traj",
          10,
          &traj_server_t::on_poly_traj,
          &traj_server,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_n3ctrl_state = nh.subscribe<n3ctrl::N3CtrlState>(
          "/n3ctrl/n3ctrl_state",
          10,
          &traj_server_t::on_n3ctrl_state,
          &traj_server,
          ros::TransportHints().tcpNoDelay()
  );

  ros::Subscriber sub_trigger = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger",10,&traj_server_t::on_trigger,&traj_server);

  ros::spin();

  ros::shutdown();

  return 0;

}
