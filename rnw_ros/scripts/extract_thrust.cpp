//
// Created by sheep on 2021/6/28.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Joy.h>
#include <fstream>

using namespace std;


int main( int argc, char** argv ){

  std::string bag_dir = "/home/sheep/Dropbox/mphil_bags";
  std::string bag_name = "2021-06-06-14-30-29.zigzag.bag";
  std::string result_dir = "/home/sheep/";

  rosbag::Bag bag;
  {
    stringstream ss; ss << bag_dir << "/" << bag_name;
    bag.open(ss.str());
  }

  stringstream ss_out;
  ss_out << result_dir << "/" << bag_name << ".thrust.csv";
  ofstream ofs(ss_out.str());

  std::vector<std::string> topics;
  topics.emplace_back("/djiros/ctrl");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool init = false;
  ros::Time init_time;

  for (rosbag::MessageInstance const m: view) {
    sensor_msgs::Joy::ConstPtr i = m.instantiate<sensor_msgs::Joy>();
    if (i == nullptr) continue;
    if ( !init ) {
      init = true;
      init_time = i->header.stamp;
    }

    double t = (i->header.stamp - init_time).toSec();
    double thrust = i->axes.at(2);

    ofs << t << "," << thrust << endl;

  }

  ofs.close();
  bag.close();

}