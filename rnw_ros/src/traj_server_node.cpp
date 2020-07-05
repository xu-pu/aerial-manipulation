#include "rnw_ros/traj_uitls.h"
#include "rnw_ros/pose_utils.h"

PolynomialTraj traj;

ros::Publisher pub_path_setpoint;

void pub_setpoint(){

  static ros::Time init_time;
  static bool init = false;

  static nav_msgs::Path path;
  path.header.frame_id = "world";

  if (!init) {
    init_time = ros::Time::now();
    init = true;
  }

  auto cur_time = ros::Time::now();
  double dt = ( cur_time - init_time ).toSec();

  if ( dt < traj.getTimeSum() ) {

    Vector3d x = traj.evaluate(dt);
    Vector3d x_dot = traj.evaluateVel(dt);
    Vector3d x_dot_dot = traj.evaluateAcc(dt);

    ROS_INFO_STREAM(dt);
    ROS_INFO_STREAM(x);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cur_time;
    pose.pose.position = eigen2ros(x);

    path.poses.push_back(pose);

    pub_path_setpoint.publish(path);

  }
  else {
    ROS_INFO_STREAM("Traj Complete");
  }

}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"traj_server_node");

  ros::NodeHandle nh("~");

  pub_path_setpoint = nh.advertise<nav_msgs::Path>("/traj/setpoint",10);

//  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>("imu",10,on_imu);
//  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("vicon",10,on_odom);
//  ros::Subscriber sub_vins = nh.subscribe<nav_msgs::Odometry>("vins",10,on_vins);

  vector<Vector3d> waypoints = gen_waypoint_zigzag(5,0.25,0.5);

  MatrixXd POS = toXd(waypoints);
  VectorXd TIMES = gen_time_intervals(1,waypoints);
  PolynomialTraj test_traj = minSnapTraj(POS,Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),TIMES);

  traj = test_traj;

  ros::Rate rate(100);

  while (ros::ok()) {

    pub_setpoint();

    ros::spinOnce();

    rate.sleep();

  }

  ros::shutdown();

  return 0;

}