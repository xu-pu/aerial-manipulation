#include "n3ctrl/controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/format.hpp>
#include <n3ctrl/ControllerDebug.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller(Parameter_t& param_):
	param(param_), vel_err_integral(param_)
{
	is_configured = false;
  vel_err_integral.reset();
}

void Controller::config()
{
	config_gain(param.hover_gain);
	is_configured = true;
}

void Controller::config_gain(const Parameter_t::Gain& gain)
{
	Kp.setZero();
	Kv.setZero();
	Kvi.setZero();
	Ka.setZero();
	Kap.setZero();
	Kp(0,0) = gain.Kp0;
	Kp(1,1) = gain.Kp1;
	Kp(2,2) = gain.Kp2;
	Kv(0,0) = gain.Kv0;
	Kv(1,1) = gain.Kv1;
	Kv(2,2) = gain.Kv2;
	Kvi(0,0) = gain.Kvi0;
	Kvi(1,1) = gain.Kvi1;
	Kvi(2,2) = gain.Kvi2;
	Ka(0,0) = gain.Ka0;
	Ka(1,1) = gain.Ka1;
	Ka(2,2) = gain.Ka2;
  Kap(0,0) = gain.Kap0;
  Kap(1,1) = gain.Kap1;
  Kap(2,2) = gain.Kap2;
	Kyaw = gain.Kyaw;
  ROS_WARN_STREAM("[n3ctrl] Gains: " << gain);
//  int_e_v.setZero();
//  ROS_INFO("[n3ctrl] integration terms reset!");
}

void Controller::update(const Desired_State_t& des,const Odom_Data_t& odom,const Imu_Data_t& imu,Controller_Output_t& u,SO3_Controller_Output_t& u_so3){

  // do not run the actual controller when the quadrotor is not armed
  if ( flight_status == flight_status_e::STOPED || disarm(des,odom) ) {
    u.roll  = 0;
    u.pitch = 0;
    u.thrust = 0;
    u.mode = Controller_Output_t::VERT_THRU;
    u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
    u.yaw = 0;
    return;
  }

  double yaw_curr = get_yaw_from_quaternion(odom.q);
  Matrix3d wRc = rotz(yaw_curr); // intermediate frame or control frame, where control gains are defined

  Vector3d cmd_vel = position_loop(des,odom);

  Vector3d cmd_acc = velocity_loop(cmd_vel,des,odom);

  Vector3d cmd_trust = Vector3d(0, 0, param.mass * param.gra) + param.mass * cmd_acc;

  Vector3d thrust_vector = regulate_cmd_thrust(cmd_trust);

	{	// n3 api control in forward-left-up frame
		Vector3d F_c = wRc.transpose() * thrust_vector; // des_f in intermediate frame
		Matrix3d wRb_odom = odom.q.toRotationMatrix();
		Vector3d z_b_curr = wRb_odom.col(2);
		double u1 = thrust_vector.dot(z_b_curr);
		u.roll  = std::atan2(-F_c.y(), F_c.z());
		u.pitch = std::atan2( F_c.x(), F_c.z());
		u.thrust = u1 / param.full_thrust;
		u.mode = Controller_Output_t::VERT_THRU;
	}

	////////////////// yaw control //////////////////////
  if(param.use_yaw_rate_ctrl){
    // calculate yaw command
    double e_yaw = des.yaw - yaw_curr;
    while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
    while(e_yaw < -M_PI) e_yaw += (2 * M_PI);
    double u_yaw = Kyaw * e_yaw;
    u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
    u.yaw = u_yaw;
  }
  else{
    u.yaw_mode = Controller_Output_t::CTRL_YAW;
    u.yaw = des.yaw;
  }

  ////////////////// debug info //////////////////////

  dbg_msg.ref_p = to_vector3_msg(des.p);
  dbg_msg.ref_v = to_vector3_msg(des.v);
  dbg_msg.ref_a = to_vector3_msg(des.a);

  dbg_msg.plant_p = to_vector3_msg(odom.p);
  dbg_msg.plant_v = to_vector3_msg(odom.v);
  dbg_msg.plant_a = to_vector3_msg(odom.q*imu.a);

  dbg_msg.cmd_v = to_vector3_msg(cmd_vel);
  dbg_msg.cmd_a = to_vector3_msg(cmd_acc);

  dbg_msg.int_v = to_vector3_msg(vel_err_integral.error_integral);

};

void Controller::publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp, const ros::Time& extra_stamp)
{
	sensor_msgs::Joy msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FRD");

	// need to translate to forward-right-down frame
	msg.axes.push_back(toDeg(u.roll));
	msg.axes.push_back(toDeg(-u.pitch));
	if (u.mode < 0)
	{
		msg.axes.push_back(u.thrust);
	}
	else
	{
		msg.axes.push_back(u.thrust*100);	
	}
	msg.axes.push_back(toDeg(-u.yaw));
	msg.axes.push_back(u.mode);
	msg.axes.push_back(u.yaw_mode);

	//add time stamp for debug
    msg.buttons.push_back(100000);
    msg.buttons.push_back(extra_stamp.sec/msg.buttons[0]);
    msg.buttons.push_back(extra_stamp.sec%msg.buttons[0]);
    msg.buttons.push_back(extra_stamp.nsec/msg.buttons[0]);
    msg.buttons.push_back(extra_stamp.nsec%msg.buttons[0]);
	
    ctrl_pub.publish(msg);

    if ( param.pub_debug_msgs ) {
      dbg_msg.header.stamp = ros::Time::now();
      dbg_msg.header.frame_id = "world";
      dbg_msg.full_thrust = param.full_thrust;
      dbg_msg.disturbance = to_vector3_msg(vel_err_integral.disturbance);
      pub_dbg_info.publish(dbg_msg);
    }

  geometry_msgs::Vector3Stamped msg_disturbance;
  msg_disturbance.header.stamp = ros::Time::now();
  msg_disturbance.header.frame_id = "world";
  msg_disturbance.vector = to_vector3_msg(vel_err_integral.disturbance);
  pub_disturbance.publish(msg_disturbance);

}

Eigen::Vector3d Controller::position_loop( const Desired_State_t& des,const Odom_Data_t& odom ){
  double yaw_curr = get_yaw_from_quaternion(odom.q);
  Matrix3d wRc = rotz(yaw_curr);
  Matrix3d cRw = wRc.transpose();
  Vector3d e_p = des.p - odom.p;
  return des.v + wRc * Kp * cRw * e_p;
}

Eigen::Vector3d Controller::velocity_loop( Eigen::Vector3d const & cmd_vel, const Desired_State_t& des,const Odom_Data_t& odom ){

  double yaw_curr = get_yaw_from_quaternion(odom.q);
  Matrix3d wRc = rotz(yaw_curr);
  Matrix3d cRw = wRc.transpose();

  Vector3d e_v = cmd_vel - odom.v;
  //Vector3d e_a = des.a - ;
  Vector3d e_a = Vector3d::Zero();

  Eigen::Vector3d u_v_i = vel_err_integral.update(e_v).output(Kvi,cRw); // can be turn off by setting zero gains
  Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
  Eigen::Vector3d u_v_d = wRc * Ka * cRw * e_a;
  Eigen::Vector3d u_v = u_v_p + u_v_i + u_v_d; // PID

  return u_v + des.a;

}

Eigen::Vector3d Controller::regulate_cmd_thrust(Eigen::Vector3d const & cmd ){

  Vector3d F_des = cmd;

  std::string constraint_info;

  if (F_des(2) < 0.5 * param.mass * param.gra){
    constraint_info = boost::str(boost::format("thrust too low F_des(2)=%.3f; ")% F_des(2));
    F_des = F_des / F_des(2) * (0.5 * param.mass * param.gra);
  }
  else if (F_des(2) > 2 * param.mass * param.gra){
    constraint_info = boost::str(boost::format("thrust too high F_des(2)=%.3f; ")% F_des(2));
    F_des = F_des / F_des(2) * (2 * param.mass * param.gra);
  }

  if (std::fabs(F_des(0)/F_des(2)) > std::tan(toRad(50.0))){
    constraint_info += boost::str(boost::format("x(%f) too tilt; ") % toDeg(std::atan2(F_des(0),F_des(2))));
    F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(toRad(30.0));
  }

  if (std::fabs(F_des(1)/F_des(2)) > std::tan(toRad(50.0))){
    constraint_info += boost::str(boost::format("y(%f) too tilt; ") % toDeg(std::atan2(F_des(1),F_des(2))));
    F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(toRad(30.0));
  }

  if ( !constraint_info.empty() ) {
    ROS_WARN_STREAM(constraint_info);
  }

  return F_des;

}

Eigen::Vector3d Controller::acceleration_loop( Eigen::Vector3d const & F_des, const Imu_Data_t& imu, const Odom_Data_t& odom ){
  // PI Controller for Acceleration
  // apply gains in the body frame
  Quaterniond Rbw = imu.q.inverse();
  Vector3d a_des = Rbw * F_des / param.mass; // body frame
  Vector3d a_est = imu.a; // body frame
  Vector3d e_a = a_des - a_est; // body frame
  if(param.pub_debug_msgs) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector = uav_utils::to_vector3_msg(e_a);
    //ctrl_dbg_e_a_pub.publish(msg);
  }
  Vector3d u = F_des + ( imu.q * ( Kap * e_a ) ) * param.mass; // world frame
  return u;
}

void Controller::on_flight_status( std_msgs::UInt8ConstPtr const & msg ){
  flight_status = (flight_status_e)msg->data;
}

bool Controller::can_disarm(const Odom_Data_t &odom) const {
  return odom.p.z() <= param.disarm.max_height;
}

bool Controller::want_to_disarm(const Desired_State_t &des) const {
  return des.p.z() < 0;
}

bool Controller::disarm( const Desired_State_t &des, const Odom_Data_t &odom ) const {
  return param.disarm.enable && can_disarm(odom) && want_to_disarm(des);
}

Eigen::Vector3d Controller::external_force_estimate(){
  return Eigen::Vector3d::Zero();
}
