#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

using namespace std;

using Eigen::Vector3d;

struct poly_traj_t {

    static constexpr int DIM_X = 0;
    static constexpr int DIM_Y = 1;
    static constexpr int DIM_Z = 2;

    quadrotor_msgs::PolynomialTrajectory msg;

    // configuration for trajectory

    int n_segment = 0;

    int traj_id = 0;

    uint32_t traj_flag = 0;

    Eigen::VectorXd times;

    Eigen::MatrixXd coefs[3];

    vector<int> orders;

    double mag_coeff;

    ros::Time final_time = ros::TIME_MIN;
    ros::Time start_time = ros::TIME_MAX;

    /**
     * This is the max value of t for eval(), regardless of mag_coeff
     * @return
     */
    double poly_duration() const {
      return (final_time - start_time).toSec();
    }

    /**
     * trajectory execution time, beware of mag_coeff
     * @return
     */
    double duration() const {
      return (final_time - start_time).toSec() * mag_coeff;
    }

    double start_yaw = 0.0;
    double final_yaw = 0.0;

    poly_traj_t() = default;

    explicit poly_traj_t( quadrotor_msgs::PolynomialTrajectory const & traj ){

      msg = traj;

      traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      traj_id = traj.trajectory_id;
      n_segment = traj.num_segment;
      final_time = start_time = traj.header.stamp;
      times.resize(n_segment);

      orders.clear();
      for (int idx = 0; idx < n_segment; ++idx){
        final_time += ros::Duration(traj.time[idx]);
        times(idx) = traj.time[idx];
        orders.push_back(traj.order[idx]);
      }

      start_yaw = traj.start_yaw;
      final_yaw = traj.final_yaw;
      mag_coeff = traj.mag_coeff;

      int max_order = *max_element(begin(orders), end(orders));

      coefs[DIM_X] = Eigen::MatrixXd::Zero(max_order + 1, n_segment);
      coefs[DIM_Y] = Eigen::MatrixXd::Zero(max_order + 1, n_segment);
      coefs[DIM_Z] = Eigen::MatrixXd::Zero(max_order + 1, n_segment);

      //ROS_WARN("stack the coefficients");
      int shift = 0;
      for (int idx = 0; idx < n_segment; ++idx){
        int order = traj.order[idx];
        for (int j = 0; j < (order + 1); ++j){
          coefs[DIM_X](j, idx) = traj.coef_x[shift + j];
          coefs[DIM_Y](j, idx) = traj.coef_y[shift + j];
          coefs[DIM_Z](j, idx) = traj.coef_z[shift + j];
        }
        shift += (order + 1);
      }

    }

    /**
     * Convert from relative time to segment time
     * @param t - seconds since the starting point
     * @param idx - index of the current segment
     * @param dt - normalized time inside the segment, [0,1]
     */
    void calc_segment_t(double t, int & idx, double & dt){
      dt = min(t, poly_duration());
      for ( idx = 0; idx < n_segment; ++idx) {
        // find the segment idx
        if (dt > times[idx] && idx + 1 < n_segment) {
          dt -= times[idx];
        }
        else {
          dt /= times[idx];
          return;
        }
      }
      // the end point
      idx = n_segment-1;
      dt = 1;
    }

    /**
     * Evaluate the polynomial at idx-th segment and time t
     * @param idx - segment idx
     * @param t - segment time [0,1]
     * @param x
     * @param x_dot
     * @param x_dot_dot
     * @param y
     * @param y_dot
     * @param y_dot_dot
     * @param z
     * @param z_dot
     * @param z_dot_dot
     */
    void eval( int idx, double t,
               double & x, double & x_dot, double & x_dot_dot,
               double & y, double & y_dot, double & y_dot_dot,
               double & z, double & z_dot, double & z_dot_dot)
    {

      x = 0.0;
      y = 0.0;
      z = 0.0;
      x_dot = 0.0;
      y_dot = 0.0;
      z_dot = 0.0;
      x_dot_dot = 0.0;
      y_dot_dot = 0.0;
      z_dot_dot = 0.0;

      // calculate x, x_dot, x_dot_dot

      int cur_order = orders[idx];
      int cur_poly_num = cur_order + 1;

      for (int i = 0; i < cur_poly_num; i++) {
        x += coefs[DIM_X].col(idx)(i) * pow(t, i);
        y += coefs[DIM_Y].col(idx)(i) * pow(t, i);
        z += coefs[DIM_Z].col(idx)(i) * pow(t, i);
        if (i < (cur_poly_num - 1)){
          x_dot += (i + 1) * coefs[DIM_X].col(idx)(i + 1) * pow(t, i) / times[idx];
          y_dot += (i + 1) * coefs[DIM_Y].col(idx)(i + 1) * pow(t, i) / times[idx];
          z_dot += (i + 1) * coefs[DIM_Z].col(idx)(i + 1) * pow(t, i) / times[idx];
        }
        if (i < (cur_poly_num - 2)){
          x_dot_dot += (i + 2) * (i + 1) * coefs[DIM_X].col(idx)(i + 2) * pow(t, i) / times[idx] / times[idx];
          y_dot_dot += (i + 2) * (i + 1) * coefs[DIM_Y].col(idx)(i + 2) * pow(t, i) / times[idx] / times[idx];
          z_dot_dot += (i + 2) * (i + 1) * coefs[DIM_Z].col(idx)(i + 2) * pow(t, i) / times[idx] / times[idx];
        }
      }

    }

    Vector3d eval_pos( double T ){

      int idx;
      double t;
      calc_segment_t(T,idx,t);

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      // calculate x, x_dot, x_dot_dot

      int cur_order = orders[idx];
      int cur_poly_num = cur_order + 1;

      for (int i = 0; i < cur_poly_num; i++) {
        x += coefs[DIM_X].col(idx)(i) * pow(t, i);
        y += coefs[DIM_Y].col(idx)(i) * pow(t, i);
        z += coefs[DIM_Z].col(idx)(i) * pow(t, i);
      }

      return {x,y,z};

    }

    Vector3d eval_acc( double T ){

      int idx;
      double t;
      calc_segment_t(T,idx,t);

      double x_dot_dot = 0.0;
      double y_dot_dot = 0.0;
      double z_dot_dot = 0.0;

      // calculate x, x_dot, x_dot_dot

      int cur_order = orders[idx];
      int cur_poly_num = cur_order + 1;

      for (int i = 0; i < cur_poly_num; i++) {
        if (i < (cur_poly_num - 2)){
          x_dot_dot += (i + 2) * (i + 1) * coefs[DIM_X].col(idx)(i + 2) * pow(t, i) / times[idx] / times[idx];
          y_dot_dot += (i + 2) * (i + 1) * coefs[DIM_Y].col(idx)(i + 2) * pow(t, i) / times[idx] / times[idx];
          z_dot_dot += (i + 2) * (i + 1) * coefs[DIM_Z].col(idx)(i + 2) * pow(t, i) / times[idx] / times[idx];
        }
      }

      return {x_dot_dot,y_dot_dot,z_dot_dot};

    }


    void gen_pos_cmd( quadrotor_msgs::PositionCommand & _cmd, const nav_msgs::Odometry & _odom ){

      _cmd.header.stamp = _odom.header.stamp;

      _cmd.header.frame_id = "odom";
      _cmd.trajectory_flag = traj_flag;
      _cmd.trajectory_id = traj_id;

      double _t = max(0.0, (_odom.header.stamp - start_time).toSec()) / mag_coeff;

      //cout<<"t: "<<t<<endl;

      // #3. calculate the desired states
      //ROS_WARN("[SERVER] the time : %.3lf\n, n = %d, m = %d", t, _n_order, _n_segment);

      int idx; double t;
      calc_segment_t(_t,idx,t);

      eval(idx,t,
           _cmd.position.x,_cmd.position.y,_cmd.position.z,
           _cmd.velocity.x,_cmd.velocity.y,_cmd.velocity.z,
           _cmd.acceleration.x,_cmd.acceleration.y,_cmd.acceleration.z);

      //_cmd.yaw = _start_yaw + (_final_yaw - _start_yaw) * t / ((_final_time - _start_time).toSec() + 1e-9);
      _cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x);

      gen_yaw_dot(_cmd,_odom);

    }

    static void gen_yaw_dot( quadrotor_msgs::PositionCommand & _cmd, nav_msgs::Odometry const & _odom ){

      //calculate proportional term of yaw_dot
      double k_p = 1.5;
      double yaw_dot_p = 0.0;
      tf::Quaternion quat(_odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);
      tf::Matrix3x3 rotM(quat);
      double roll, pitch, yaw;
      rotM.getRPY(roll, pitch, yaw);
      const double pi = 3.14159265358979;
      double deltaYaw = yaw - _cmd.yaw;
      if (deltaYaw <= -pi){
        deltaYaw += 2 * pi;
      }
      if (deltaYaw > pi) {
        deltaYaw -= 2 * pi;
      }
      yaw_dot_p = -k_p * deltaYaw;

      //calculate derivative term of yaw_dot
      double k_d = 0.25;
      double yaw_dot_d = 0.0;
      double v_horiz_sqr_norm = _cmd.velocity.y * _cmd.velocity.y + _cmd.velocity.x * _cmd.velocity.x;
      yaw_dot_d = v_horiz_sqr_norm == 0 ? 0 : k_d * (-_cmd.velocity.y * _cmd.acceleration.x + _cmd.velocity.x * _cmd.acceleration.y) / v_horiz_sqr_norm;

      //synthesis yaw_dot
      _cmd.yaw_dot = yaw_dot_p + yaw_dot_d;

      if (fabs(_cmd.yaw_dot) > pi / 3){
        _cmd.yaw_dot /= fabs(_cmd.yaw_dot);
        _cmd.yaw_dot *= pi / 3;
      }

      if(_cmd.velocity.z*_cmd.velocity.z * 2.0 > _cmd.velocity.y*_cmd.velocity.y + _cmd.velocity.x*_cmd.velocity.x){
        _cmd.yaw = yaw;
        _cmd.yaw_dot = 0.0;
      }

    }




};

struct traj_visualizer_t {

    static constexpr double clear_after_n_sec = 1;

    quadrotor_msgs::PolynomialTrajectory latest_msg;

    poly_traj_t poly_traj;

    bool init = false;

    ros::Publisher pub_marker_traj;
    ros::Publisher pub_marker_acc;

    explicit traj_visualizer_t( ros::NodeHandle & nh ) {

      pub_marker_traj = nh.advertise<visualization_msgs::Marker>("/markers/traj", 1);
      pub_marker_acc = nh.advertise<visualization_msgs::MarkerArray>("/markers/acc", 10);

    }

    void on_traj(  quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg  ){
      latest_msg = *msg;
      poly_traj = poly_traj_t(latest_msg);
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

    void clear_markers(){

    }

};

void on_spin(){}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_traj_node");

  ros::NodeHandle nh("~");

  traj_visualizer_t traj_viz(nh);

  constexpr size_t spin_hz = 10;

  auto timer = nh.createTimer( ros::Duration( 1.0 / spin_hz ), &traj_visualizer_t::on_spin, &traj_viz );

  ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>(
          "/poly_traj_test", 100, &traj_visualizer_t::on_traj, &traj_viz );

  ros::spin();

  traj_viz.clear_markers();

  ros::shutdown();

  return 0;

}