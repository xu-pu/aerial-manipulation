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
    if ( ptr->header.stamp >= v_cone_state.at(idx).header.stamp ) {
      dst.at(idx) = *ptr;
      idx++;
    }
    if ( idx >= dst.size() ) {
      break;
    }
  }

  cout << "topic" << name << ", " << idx << " synced" << endl;

}

//void sync_all_data( rosbag::Bag & bag ){
//  sync_topic<rnw_msgs::ConeState>(bag,"/cone/state",v_cone_state);
//  sync_topic<nav_msgs::Odometry>(bag,"/drone1/odom",v_drone1_odom);
//  sync_topic<nav_msgs::Odometry>(bag,"/drone2/odom",v_drone2_odom);
//}
//
//void gen_csv( string const & name ){
//
//  std::string result_dir = "/home/sheep/";
//  stringstream ss; ss << result_dir << "/" << name << ".full.csv";
//  ofstream ofs(ss.str());
//
//  for ( size_t i=0; i<v_rnw_state.size(); i++ ) {
//    ofs << (v_rnw_state.at(i).header.stamp - start_time).toSec() << ","
//        << v_cone_state.at(i).euler_angles.x << ","
//        << v_cone_state.at(i).euler_angles.y << ","
//        << v_cone_state.at(i).euler_angles.z << ","
//        << v_cone_state.at(i).euler_angles_velocity.x << ","
//        << v_cone_state.at(i).euler_angles_velocity.y << ","
//        << v_cone_state.at(i).euler_angles_velocity.z << ","
//        << v_cone_state.at(i).contact_point.x << ","
//        << v_cone_state.at(i).contact_point.y << ",";
//    if (!v_drone1_ctrl.empty()) {
//      ofs << "," << v_drone1_ctrl.at(i).axes.at(2);
//    }
//    ofs << endl;
//  }
//
//  ofs.close();
//
//}

int main( int argc, char** argv ) {

  std::string bag_dir = "/home/sheep/Dropbox/rl_bags";
  std::string bag_name = "2021-09-01-18-31-38.labfloor.01.bag";

  rosbag::Bag bag;
  stringstream ss; ss << bag_dir << "/" << bag_name;
  bag.open(ss.str());

  extract_start_time(bag);

  extract_end_time(bag);

  extract_cone_state(bag);

//  sync_all_data(bag);
//
//  gen_csv(bag_name);

  bag.close();

}