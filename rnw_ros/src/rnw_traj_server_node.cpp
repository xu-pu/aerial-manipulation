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

Quaterniond ros2eigen( geometry_msgs::Quaternion const & quat ){
  return { quat.w, quat.x, quat.y, quat.z };
}

Vector3d ros2eigen( geometry_msgs::Point const & pos ){
  return { pos.x, pos.y, pos.z };

}

struct traj_server_t {

    geometry_msgs::PoseStamped trigger_pose;

    uint32_t traj_id;

    PolynomialTraj traj;

    bool traj_available = false;

    double base_yaw;

    nav_msgs::Path path_setpoint;

    nav_msgs::Path path_plant;

    ros::Time traj_start_time;

    bool traj_started = false;

    bool triggered = false;

    explicit traj_server_t(){
      path_setpoint.header.frame_id = "world";
      path_plant.header.frame_id = "world";
    }

    void on_planning( OdometryConstPtr const & msg ){

      traj_id = trigger_pose.header.seq;

      Matrix3d R = ros2eigen(msg->pose.pose.orientation).toRotationMatrix();
      Vector3d T = ros2eigen(msg->pose.pose.position);

      base_yaw = quat2eulers(Quaterniond(R)).z();

      vector<Vector3d> waypoints = gen_waypoint_zigzag(5,0.25,0.5);
      vector<Vector3d> wps = transform_pts(waypoints,R,T);

      traj = minsnap(wps,3);

      traj_available = true;

    }

    void on_trigger( geometry_msgs::PoseStampedConstPtr const & msg ){

      on_cleanup();

      trigger_pose = *msg;

      triggered = true;

    }

    void on_cleanup(){
      path_setpoint.poses.clear();
      path_plant.poses.clear();
      triggered = false;
      traj_started = false;
      traj_available = false;
    }

    void on_odom( OdometryConstPtr const & odom ){

      if ( triggered ){
        triggered = false;
        on_planning(odom);
      }

      if ( traj_available ) {

        double dt = this->dt();

        if ( dt <  traj.getTimeSum() ){
          Vector3d x = traj.evaluate(dt);
          Vector3d x_dot = traj.evaluateVel(dt);
          Vector3d x_dot_dot = traj.evaluateAcc(dt);

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
          cmd.yaw = base_yaw;
          cmd.yaw_dot = 0;
          cmd.trajectory_flag = cmd.TRAJECTORY_STATUS_READY;
          cmd.trajectory_id = traj_id;

          pub_pos_cmd.publish(cmd);

        }
        else {
          ROS_INFO_STREAM("Traj Complete");
          on_cleanup();
        }


      }

    }

    double dt() {
      if (!traj_started) {
        traj_start_time = ros::Time::now();
        traj_started = true;
      }
      return (ros::Time::now()-traj_start_time).toSec();
    }

};

traj_server_t traj_server;

void on_odom( OdometryConstPtr const & odom ) {
  traj_server.on_odom(odom);
}

void on_trigger( geometry_msgs::PoseStampedConstPtr const & msg ){
  ROS_INFO_STREAM("Traj Server Triggered, traj_id: " << msg->header.seq);
  traj_server.on_trigger(msg);
}

void on_trigger_tip(std_msgs::HeaderConstPtr const & msg ){
  ROS_INFO_STREAM("Trigger: Go to the tip");
}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"rnw_traj_server_node");

  ros::NodeHandle nh("~");

  pub_path_setpoint = nh.advertise<nav_msgs::Path>("/traj/setpoint",10);
  pub_path_plant = nh.advertise<nav_msgs::Path>("/traj/plant",10);
  pub_pos_cmd = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",10);

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/uwb_vicon_odom",10,on_odom);
  ros::Subscriber sub_trigger = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger",10,on_trigger);

  ros::Subscriber sub_trigger_tip = nh.subscribe<std_msgs::Header>("/rnw/triggers/tip", 10, on_trigger_tip);

  ros::spin();

  ros::shutdown();

  return 0;

}
