#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

using namespace std;

struct poly_traj_t {

    static constexpr int _DIM_x = 0;
    static constexpr int _DIM_y = 1;
    static constexpr int _DIM_z = 2;

    quadrotor_msgs::PolynomialTrajectory msg;

    // configuration for trajectory
    int _n_segment = 0;
    int _traj_id = 0;
    uint32_t _traj_flag = 0;
    Eigen::VectorXd _time;
    Eigen::MatrixXd _coef[3];
    vector<int> _order;
    double mag_coeff;

    ros::Time _final_time = ros::TIME_MIN;
    ros::Time _start_time = ros::TIME_MAX;

    double _start_yaw = 0.0, _final_yaw = 0.0;

    poly_traj_t() = default;

    explicit poly_traj_t( quadrotor_msgs::PolynomialTrajectory const & traj ){

      msg = traj;

      _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      _traj_id = traj.trajectory_id;
      _n_segment = traj.num_segment;
      _final_time = _start_time = traj.header.stamp;
      _time.resize(_n_segment);

      _order.clear();
      for (int idx = 0; idx < _n_segment; ++idx)
      {
        _final_time += ros::Duration(traj.time[idx]);
        _time(idx) = traj.time[idx];
        _order.push_back(traj.order[idx]);
      }

      _start_yaw = traj.start_yaw;
      _final_yaw = traj.final_yaw;
      mag_coeff = traj.mag_coeff;

      int max_order = *max_element(begin(_order), end(_order));

      _coef[_DIM_x] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
      _coef[_DIM_y] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
      _coef[_DIM_z] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);

      //ROS_WARN("stack the coefficients");
      int shift = 0;
      for (int idx = 0; idx < _n_segment; ++idx)
      {
        int order = traj.order[idx];

        for (int j = 0; j < (order + 1); ++j)
        {
          _coef[_DIM_x](j, idx) = traj.coef_x[shift + j];
          _coef[_DIM_y](j, idx) = traj.coef_y[shift + j];
          _coef[_DIM_z](j, idx) = traj.coef_z[shift + j];
        }

        shift += (order + 1);
      }


    }

    void gen_pos_cmd( quadrotor_msgs::PositionCommand & _cmd, const nav_msgs::Odometry & _odom ){

      _cmd.header.stamp = _odom.header.stamp;

      _cmd.header.frame_id = "odom";
      _cmd.trajectory_flag = _traj_flag;
      _cmd.trajectory_id = _traj_id;

      double t = max(0.0, (_odom.header.stamp - _start_time).toSec()) / mag_coeff;

      //cout<<"t: "<<t<<endl;

      // #3. calculate the desired states
      //ROS_WARN("[SERVER] the time : %.3lf\n, n = %d, m = %d", t, _n_order, _n_segment);
      for (int idx = 0; idx < _n_segment; ++idx) {
        // for each segment
        if (t > _time[idx] && idx + 1 < _n_segment) {
          t -= _time[idx];
        }
        else {
          t /= _time[idx];

          _cmd.position.x = 0.0;
          _cmd.position.y = 0.0;
          _cmd.position.z = 0.0;
          _cmd.velocity.x = 0.0;
          _cmd.velocity.y = 0.0;
          _cmd.velocity.z = 0.0;
          _cmd.acceleration.x = 0.0;
          _cmd.acceleration.y = 0.0;
          _cmd.acceleration.z = 0.0;

          int cur_order = _order[idx];
          int cur_poly_num = cur_order + 1;

          for (int i = 0; i < cur_poly_num; i++)
          {
            _cmd.position.x += _coef[_DIM_x].col(idx)(i) * pow(t, i);
            _cmd.position.y += _coef[_DIM_y].col(idx)(i) * pow(t, i);
            _cmd.position.z += _coef[_DIM_z].col(idx)(i) * pow(t, i);

            if (i < (cur_poly_num - 1))
            {
              _cmd.velocity.x += (i + 1) * _coef[_DIM_x].col(idx)(i + 1) * pow(t, i) / _time[idx];

              _cmd.velocity.y += (i + 1) * _coef[_DIM_y].col(idx)(i + 1) * pow(t, i) / _time[idx];

              _cmd.velocity.z += (i + 1) * _coef[_DIM_z].col(idx)(i + 1) * pow(t, i) / _time[idx];
            }

            if (i < (cur_poly_num - 2))
            {
              _cmd.acceleration.x += (i + 2) * (i + 1) * _coef[_DIM_x].col(idx)(i + 2) * pow(t, i) / _time[idx] / _time[idx];

              _cmd.acceleration.y += (i + 2) * (i + 1) * _coef[_DIM_y].col(idx)(i + 2) * pow(t, i) / _time[idx] / _time[idx];

              _cmd.acceleration.z += (i + 2) * (i + 1) * _coef[_DIM_z].col(idx)(i + 2) * pow(t, i) / _time[idx] / _time[idx];
            }
          }

          //_cmd.yaw = _start_yaw + (_final_yaw - _start_yaw) * t / ((_final_time - _start_time).toSec() + 1e-9);
          _cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x);

          //calculate proportional term of yaw_dot
          double k_p = 1.5;
          double yaw_dot_p = 0.0;
          tf::Quaternion quat(_odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);
          tf::Matrix3x3 rotM(quat);
          double roll, pitch, yaw;
          rotM.getRPY(roll, pitch, yaw);
          const double pi = 3.14159265358979;
          double deltaYaw = yaw - _cmd.yaw;
          if (deltaYaw <= -pi)
          {
            deltaYaw += 2 * pi;
          }
          if (deltaYaw > pi)
          {
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

          if (fabs(_cmd.yaw_dot) > pi / 3)
          {
            _cmd.yaw_dot /= fabs(_cmd.yaw_dot);
            _cmd.yaw_dot *= pi / 3;
          }

          if(_cmd.velocity.z*_cmd.velocity.z * 2.0 > _cmd.velocity.y*_cmd.velocity.y + _cmd.velocity.x*_cmd.velocity.x)
          {
            _cmd.yaw = yaw;
            _cmd.yaw_dot = 0.0;
          }

          break;
        }
      }

    }

};





void on_traj( quadrotor_msgs::PolynomialTrajectoryConstPtr const & msg ) {


}

int main( int argc, char** argv ) {

  ros::init(argc,argv,"visualize_traj_node");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_traj = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>("/poly_traj_test",100,on_traj);

  ros::spin();

  ros::shutdown();

  return 0;

}