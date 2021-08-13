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

vector<rnw_msgs::RnwState> v_rnw_state;
vector<rnw_msgs::ConeState> v_cone_state;
vector<nav_msgs::Odometry> v_drone1_odom;
vector<nav_msgs::Odometry> v_drone2_odom;
vector<sensor_msgs::Joy> v_drone1_ctrl;
vector<double> v_peak_ke;

ros::Time ready_time_ground;
ros::Time ready_time_air;

double mass = 2.6;
double xCM = 0.15;
double zCM = 0.37;

void extract_ground_drone1_ready_time( rosbag::Bag & bag ) {

  std::vector<std::string> topics;
  topics.emplace_back("/drone1/state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool found = false;

  for (rosbag::MessageInstance const m: view) {
    n3ctrl::N3CtrlState::ConstPtr i = m.instantiate<n3ctrl::N3CtrlState>();
    if (i == nullptr) continue;
    if ( i->state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER ) {
      ready_time_ground = i->header.stamp;
      found = true;
      break;
    }
  }

  assert(found);

  cout << "ready_time_ground: " << ready_time_ground << endl;

}

void extract_drone1_ready_time( rosbag::Bag & bag ) {

  std::vector<std::string> topics;
  topics.emplace_back("/n3ctrl/n3ctrl_state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool found = false;

  for (rosbag::MessageInstance const m: view) {
    n3ctrl::N3CtrlState::ConstPtr i = m.instantiate<n3ctrl::N3CtrlState>();
    if (i == nullptr) continue;
    if ( i->state >= n3ctrl::N3CtrlState::STATE_CMD_HOVER ) {
      ready_time_air = i->header.stamp;
      found = true;
      break;
    }
  }

  assert(found);

  cout << "ready_time_air: " << ready_time_air << endl;

}

void extract_rnw_data( rosbag::Bag & bag ) {

  std::vector<std::string> topics;
  topics.emplace_back("/rnw/state");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  rnw_msgs::RnwState latch;
  latch.is_walking = false;

  for (rosbag::MessageInstance const m: view) {
    rnw_msgs::RnwState::ConstPtr i = m.instantiate<rnw_msgs::RnwState>();
    if (i == nullptr) continue;

    if ( i->is_walking ) {
      v_rnw_state.emplace_back(*i);
    }

    if ( !latch.is_walking && i->is_walking ) {
      // start
      start_time = i->header.stamp;
    }
    else if ( latch.is_walking && !i->is_walking ) {
      // stopped
      end_time = latch.header.stamp;
      break;
    }

    latch = *i;

  }

  cout << "rnw duration: " << (end_time-start_time).toSec() << ", count: " << v_rnw_state.size() << endl;

}

template<typename T>
void sync_topic( rosbag::Bag & bag, string const & name, std::vector<T> & dst ){

  dst.clear();
  dst.resize(v_rnw_state.size());

  std::vector<std::string> topics;
  topics.emplace_back(name);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  size_t idx = 0;

  for (rosbag::MessageInstance const m: view) {
    auto ptr = m.instantiate<T>();
    if ( ptr == nullptr ) continue;
    if ( ptr->header.stamp >= v_rnw_state.at(idx).header.stamp ) {
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
  sync_topic<rnw_msgs::ConeState>(bag,"/cone/state",v_cone_state);
  sync_topic<nav_msgs::Odometry>(bag,"/drone1/odom",v_drone1_odom);
  sync_topic<nav_msgs::Odometry>(bag,"/drone2/odom",v_drone2_odom);
}

double calc_angle( rnw_msgs::ConeState const & cone, nav_msgs::Odometry const & odom1, nav_msgs::Odometry const & odom2 ){

  Eigen::Vector3d cp = uav_utils::from_point_msg(cone.tip);
  Eigen::Vector3d d1 = uav_utils::from_point_msg(odom1.pose.pose.position);
  Eigen::Vector3d d2 = uav_utils::from_point_msg(odom2.pose.pose.position);

  Eigen::Vector3d v1 = d1-cp;
  Eigen::Vector3d v2 = d2-cp;

  return std::atan2(v1.cross(v2).norm(), v1.dot(v2));

}


void gen_csv( string const & name ){

  std::string result_dir = "/home/sheep/";
  stringstream ss; ss << result_dir << "/" << name << ".full.csv";
  ofstream ofs(ss.str());

  for ( size_t i=0; i<v_rnw_state.size(); i++ ) {
    ofs << (v_rnw_state.at(i).header.stamp - start_time).toSec() << ","
        << v_cone_state.at(i).euler_angles.x << ","
        << v_cone_state.at(i).euler_angles.y << ","
        << v_cone_state.at(i).euler_angles.z << ","
        << v_cone_state.at(i).euler_angles_velocity.x << ","
        << v_cone_state.at(i).euler_angles_velocity.y << ","
        << v_cone_state.at(i).euler_angles_velocity.z << ","
        << v_cone_state.at(i).contact_point.x << ","
        << v_cone_state.at(i).contact_point.y << ","
        << v_peak_ke.at(i) << ","
        << calc_angle(v_cone_state.at(i),v_drone1_odom.at(i),v_drone2_odom.at(i));
    if (!v_drone1_ctrl.empty()) {
      ofs << "," << v_drone1_ctrl.at(i).axes.at(2);
    }
    ofs << endl;
  }

  ofs.close();

}

vector<rnw_msgs::ConeState> extract_phi_peaks(){

  vector<rnw_msgs::ConeState> rst;

  for ( size_t i=1; i<v_cone_state.size()-1; i++ ) {

    double phi_cur = abs(v_cone_state.at(i).euler_angles.z);
    double phi_pre = abs(v_cone_state.at(i-1).euler_angles.z);
    double phi_nxt = abs(v_cone_state.at(i+1).euler_angles.z);

    if ( phi_cur > phi_nxt && phi_cur > phi_pre ) {
      rst.emplace_back(v_cone_state.at(i));
    }

  }

  return rst;

}

void gen_ke( string const & bag_name ){

  v_peak_ke.resize(v_rnw_state.size());

  stringstream ss; ss << "/home/sheep/" << bag_name << ".ke.csv";
  ofstream ofs(ss.str());

  ros::Time st = v_rnw_state.front().header.stamp;
  size_t step = 0;
  double peak_ke = 0;
  double ke = 0;

  for ( size_t i=0; i<v_rnw_state.size(); i++ ) {
    v_peak_ke.at(i) = ke;
    ofs << (v_rnw_state.at(i).header.stamp-st).toSec() << "," << ke << endl;
    peak_ke = std::max<double>(peak_ke, calc_kinetic_energy(v_cone_state.at(i),mass,xCM,zCM));
    if ( v_rnw_state.at(i).step_count > step ) {
      step = v_rnw_state.at(i).step_count;
      ke = peak_ke;
      peak_ke = 0;
    }
  }

  ofs.close();

}

void gen_amp( string const & bag_name ){

  auto rst = extract_phi_peaks();

  std::string result_dir = "/home/sheep/";
  stringstream ss; ss << result_dir << "/" << bag_name << ".peaks.csv";
  ofstream ofs(ss.str());

  for ( auto iter : rst ) {
    ofs << (iter.header.stamp-start_time).toSec() << "," << iter.euler_angles.z << endl;
  }

  ofs.close();

}

void sync_thrust( rosbag::Bag & bag_ground, rosbag::Bag & bag_air ){

  extract_ground_drone1_ready_time(bag_ground);
  extract_drone1_ready_time(bag_air);

  v_drone1_ctrl.clear();
  v_drone1_ctrl.resize(v_rnw_state.size());

  std::vector<std::string> topics;
  topics.emplace_back("/djiros/ctrl");
  rosbag::View view(bag_air, rosbag::TopicQuery(topics));

  size_t idx = 0;

  for (rosbag::MessageInstance const m: view) {
    auto ptr = m.instantiate<sensor_msgs::Joy>();
    if ( ptr == nullptr ) continue;
    ros::Time ground_time = ready_time_ground + ( ptr->header.stamp - ready_time_air );
    if ( ground_time >= v_rnw_state.at(idx).header.stamp ) {
      v_drone1_ctrl.at(idx) = *ptr;
      idx++;
    }
    if ( idx >= v_drone1_ctrl.size() ) {
      break;
    }
  }

  cout << "thrust of drone1" << ", " << idx << " synced" << endl;

}

int main( int argc, char** argv ) {

  bool add_thrust = true;

  std::string bag_dir = "/home/sheep/Dropbox/mphil_bags";
  std::string bag_name = "2021-07-01-02-40-15.table7.perfect.ground.45.120.bag";
  std::string air_bag_name = "2021-06-09-00-18-33.table7.perfect.drone1.45.120.bag";

  rosbag::Bag bag;
  stringstream ss; ss << bag_dir << "/" << bag_name;
  bag.open(ss.str());

  extract_rnw_data(bag);
  sync_all_data(bag);

  if ( add_thrust ) {
    rosbag::Bag air_bag;
    stringstream ss; ss << bag_dir << "/" << air_bag_name;
    air_bag.open(ss.str());
    sync_thrust(bag,air_bag);
  }

  gen_amp(bag_name);

  gen_ke(bag_name);

  gen_csv(bag_name);

  bag.close();

}