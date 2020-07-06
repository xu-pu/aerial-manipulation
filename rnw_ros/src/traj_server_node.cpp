#include "rnw_ros/traj_uitls.h"
#include "rnw_ros/pose_utils.h"

ros::Publisher pub_path_setpoint;
ros::Publisher pub_path_plant;
ros::Publisher pub_pos_cmd;

PolynomialTraj minsnap( vector<Vector3d> const & waypoints, double interval ){
  MatrixXd POS = toXd(waypoints);
  VectorXd TIMES = gen_time_intervals(interval,waypoints);
  return minSnapTraj(POS,Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),TIMES);
}

PoseStamped eigen2pathpoint( Vector3d const & T ){
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.pose.position = eigen2ros(T);
  return pose;
}

struct traj_server_t {

    nav_msgs::Odometry base_odom; // trajectory is defined in this reference frame, FLU

    PolynomialTraj traj;

    bool initialized = false;

    double base_yaw;

    void init( OdometryConstPtr const & msg ){

      base_odom = *msg;

      ROS_INFO_STREAM(base_odom);

      Matrix3d R = odom2R(msg);
      Vector3d T = odom2T(msg);

      base_yaw = quat2eulers(Quaterniond(R)).z();

      vector<Vector3d> waypoints = gen_waypoint_zigzag(5,0.25,0.5);

      vector<Vector3d> wps = transform_pts(waypoints,R,T);

      traj = minsnap(wps,1);

      initialized = true;

    }

    ros::Time init_t;

    bool init_t_init = false;

    double dt() {
      if (!init_t_init) {
        init_t = ros::Time::now();
        init_t_init = true;
      }
      return (ros::Time::now()-init_t).toSec();
    }

};

traj_server_t traj_server;

void on_odom( OdometryConstPtr const & odom ) {

  static nav_msgs::Path path_setpoint;
  static nav_msgs::Path path_plant;
  path_setpoint.header.frame_id = "world";
  path_plant.header.frame_id = "world";

  if ( !traj_server.initialized ) {
    traj_server.init(odom);
  }
  else {
    double dt = traj_server.dt();
    if ( dt < traj_server.traj.getTimeSum() ) {

      Vector3d x = traj_server.traj.evaluate(dt);
      Vector3d x_dot = traj_server.traj.evaluateVel(dt);
      Vector3d x_dot_dot = traj_server.traj.evaluateAcc(dt);

//      ROS_INFO_STREAM(dt);
//      ROS_INFO_STREAM(x);

      // path

      path_setpoint.poses.emplace_back(eigen2pathpoint(x));
      path_plant.poses.emplace_back(eigen2pathpoint(odom2T(odom)));

      pub_path_setpoint.publish(path_setpoint);
      pub_path_plant.publish(path_plant);

      // position command

      quadrotor_msgs::PositionCommand cmd;
      cmd.position = eigen2ros(x);
      cmd.velocity = eigen2rosv(x_dot);
      cmd.acceleration = eigen2rosv(x_dot_dot);
      cmd.yaw = traj_server.base_yaw;
      cmd.yaw_dot = 0;

      cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_READY;
      cmd.trajectory_id = 2;

      pub_pos_cmd.publish(cmd);

    }
    else {
      ROS_INFO_STREAM("Traj Complete");
    }
  }

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"traj_server_node");

  ros::NodeHandle nh("~");

  pub_path_setpoint = nh.advertise<nav_msgs::Path>("/traj/setpoint",10);
  pub_path_plant = nh.advertise<nav_msgs::Path>("/traj/plant",10);
  pub_pos_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",10);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("vicon",10,on_odom);

  ros::spin();

  ros::shutdown();

  return 0;

}