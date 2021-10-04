//
// Created by sheep on 2021/6/28.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Joy.h>
#include <fstream>
#include <rnw_msgs/RnwState.h>
#include <nav_msgs/Odometry.h>
#include <rnw_msgs/ConeState.h>
#include <Eigen/Dense>
#include <uav_utils/converters.h>
#include "rnw_ros/rnw_utils.h"
#include <n3ctrl/N3CtrlState.h>

using namespace std;

ros::Time start_time;
ros::Time end_time;

vector<rnw_msgs::ConeState> v_cone_state;
vector<sensor_msgs::Joy> v_rl_action;
vector<Eigen::Vector2d> v_cp_vel;
vector<double> v_lpf_action_x;
vector<double> v_lpf_action_y;
vector<double> v_lpf_exe_x;
vector<double> v_lpf_exe_y;

Eigen::Vector2d calc_cp_vel( rnw_msgs::ConeState const & cs_pre, rnw_msgs::ConeState const & cs_cur ){
  Matrix3d R = calc_rnw_body_frame(cs_cur);
  Vector3d T = uav_utils::from_point_msg(cs_cur.tip);
  Vector3d X = uav_utils::from_point_msg(cs_pre.tip);
  Matrix3d R_prime = R.transpose();
  Vector3d T_prime = -R_prime*T;
  Vector3d X_prime = R_prime * X + T_prime;
  double dt = (cs_cur.header.stamp - cs_pre.header.stamp).toSec();
  Vector3d vel = X_prime / dt;
  return { vel.x(), vel.y() };
}

void extract_cp_vel(){

  v_cp_vel.resize(v_cone_state.size());

  v_cp_vel.at(0) = { 0, 0 };

  for ( size_t i=1; i<v_cone_state.size(); i++ ) {
    v_cp_vel.at(i) = calc_cp_vel(v_cone_state.at(i-1),v_cone_state.at(i));
  }

}

void extract_start_time( rosbag::Bag & bag ){

  std::vector<std::string> topics;
  topics.emplace_back("/drone1/state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool high = false;

  for (rosbag::MessageInstance const m: view) {
    n3ctrl::N3CtrlState::ConstPtr i = m.instantiate<n3ctrl::N3CtrlState>();
    if (i == nullptr) continue;

    bool cur_high = i->state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER;

    if ( cur_high && !high ) {
      start_time = i->header.stamp;
      break;
    }

    high = cur_high;

  }

  cout << "start_time: " << start_time << endl;

}

void extract_end_time( rosbag::Bag & bag ){

  std::vector<std::string> topics;
  topics.emplace_back("/drone1/state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool high = false;

  for (rosbag::MessageInstance const m: view) {
    n3ctrl::N3CtrlState::ConstPtr i = m.instantiate<n3ctrl::N3CtrlState>();
    if (i == nullptr) continue;

    bool cur_high = i->state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER;

    if ( high && !cur_high ) {
      end_time = i->header.stamp;
      break;
    }

    high = cur_high;

  }

  cout << "end_time: " << end_time << endl;

}

void extract_cone_state( rosbag::Bag & bag ) {

  std::vector<std::string> topics;
  topics.emplace_back("/cone/state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  v_cone_state.clear();

  for (rosbag::MessageInstance const m: view) {
    rnw_msgs::ConeState::ConstPtr i = m.instantiate<rnw_msgs::ConeState>();
    if (i == nullptr) continue;

    if ( i->header.stamp >= start_time && i->header.stamp <= end_time ) {
      v_cone_state.emplace_back(*i);
    }

  }

  cout << "rnw duration: " << (end_time-start_time).toSec() << ", count: " << v_cone_state.size() << endl;

}

template<typename T>
void sync_topic( rosbag::Bag & bag, string const & name, std::vector<T> & dst ){

  dst.clear();
  dst.resize(v_cone_state.size());

  std::vector<std::string> topics;
  topics.emplace_back(name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  size_t idx = 0;

  for (rosbag::MessageInstance const m: view) {
    auto ptr = m.instantiate<T>();
    if ( ptr == nullptr ) continue;
    while ( idx < dst.size() && ptr->header.stamp >= v_cone_state.at(idx).header.stamp ) {
      dst.at(idx) = *ptr;
      idx++;
    }
    if ( idx >= dst.size() ) {
      break;
    }
  }

  cout << "topic" << name << ", " << idx << " synced" << endl;

}

void sync_all_data( rosbag::Bag & bag ){
  sync_topic<sensor_msgs::Joy>(bag,"/rl_agent/action",v_rl_action);
}

void gen_csv( string const & name ){

  std::string result_dir = "/home/sheep/am_ws/src/aerial_manip/data";
  stringstream ss; ss << result_dir << "/" << name << ".full.csv";
  ofstream ofs(ss.str());

  for ( size_t i=0; i<v_cone_state.size(); i++ ) {
    ofs << (v_cone_state.at(i).header.stamp - start_time).toSec() << ","
        << v_cone_state.at(i).euler_angles.x << ","
        << v_cone_state.at(i).euler_angles.y << ","
        << v_cone_state.at(i).euler_angles.z << ","
        << v_cone_state.at(i).euler_angles_velocity.x << ","
        << v_cone_state.at(i).euler_angles_velocity.y << ","
        << v_cone_state.at(i).euler_angles_velocity.z << ","
        << v_cone_state.at(i).contact_point.x << ","
        << v_cone_state.at(i).contact_point.y << ","
        << v_lpf_action_x.at(i) << ","
        << v_lpf_action_y.at(i) << ","
        << v_lpf_exe_x.at(i) << ","
        << v_lpf_exe_y.at(i);
    ofs << endl;
  }

  ofs.close();

}

void apply_lpf(){

  double T = 0.1;

  bool enable = true;

  lpf_1st_butterworth_t lpf_action_x(T);
  lpf_1st_butterworth_t lpf_action_y(T);
  lpf_1st_butterworth_t lpf_exe_x(T);
  lpf_1st_butterworth_t lpf_exe_y(T);

  v_lpf_action_x.resize(v_cone_state.size());
  v_lpf_action_y.resize(v_cone_state.size());
  v_lpf_exe_x.resize(v_cone_state.size());
  v_lpf_exe_y.resize(v_cone_state.size());

  if ( enable ) {
    for ( size_t i=0; i<v_cone_state.size(); i++ ) {
      v_lpf_action_x.at(i) = lpf_action_x.filter(v_rl_action.at(i).axes.at(0));
      v_lpf_action_y.at(i) = lpf_action_y.filter(v_rl_action.at(i).axes.at(1));
      v_lpf_exe_x.at(i) = lpf_exe_x.filter(v_cp_vel.at(i).x());
      v_lpf_exe_y.at(i) = lpf_exe_y.filter(v_cp_vel.at(i).y());
    }
  }
  else {
    for ( size_t i=0; i<v_cone_state.size(); i++ ) {
      v_lpf_action_x.at(i) = v_rl_action.at(i).axes.at(0);
      v_lpf_action_y.at(i) = v_rl_action.at(i).axes.at(1);
      v_lpf_exe_x.at(i) = v_cp_vel.at(i).x();
      v_lpf_exe_y.at(i) = v_cp_vel.at(i).y();
    }
  }

}

int main( int argc, char** argv ) {

  std::string bag_dir = "/home/sheep/Dropbox/rl_bags";
  std::string bag_name = "2021-09-02-02-23-49.foam.02.bag";

  rosbag::Bag bag;
  stringstream ss; ss << bag_dir << "/" << bag_name;
  bag.open(ss.str());

  extract_start_time(bag);

  extract_end_time(bag);

  extract_cone_state(bag);

  sync_all_data(bag);

  extract_cp_vel();

  apply_lpf();

  gen_csv(bag_name);

  bag.close();

}