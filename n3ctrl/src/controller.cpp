#include "n3ctrl/controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/format.hpp>
#include <n3ctrl/ControllerDebug.h>
#include <djiros/DroneArmControl.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller(Parameter_t& param_):
	param(param_), vel_err_integral(param_)
{
  // param is not loaded here
	is_configured = false;
  vel_err_integral.reset();
}

void Controller::config()
{
	config_gain(param.hover_gain);
	is_configured = true;
  lpf_acc.T = param.indi_lpf;
  lpf_thrust.T = param.indi_lpf;
  if ( !lpf_thrust.initialized ) {
    lpf_thrust.filter(Vector3d::UnitZ()*param.gra);
  }
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

  Eigen::Vector3d vg = - Vector3d::UnitZ() * param.gra;

  // lpf acc
  lpf_acc.filter( imu.q * imu.a + vg );

  // do not run the actual controller when the quadrotor is not armed
  if ( flight_status == flight_status_e::STOPED ) {
    //ROS_WARN_STREAM("[n3ctrl] not armed!");
    u.roll  = 0;
    u.pitch = 0;
    u.thrust = 0.1;
    u.mode = Controller_Output_t::VERT_THRU;
    u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
    u.yaw = 0;
    return;
  }
  else if (should_disarm(des, odom)) {
    ROS_WARN_STREAM("[n3ctrl] should_disarm!");
    disarm();
    u.roll  = 0;
    u.pitch = 0;
    u.thrust = 0;
    u.mode = Controller_Output_t::VERT_THRU;
    u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
    u.yaw = 0;
    return;
  }
  else if ( sec_since_motor_armed() < param.wait_before_take_off ) {
    // count down
    u.roll  = 0;
    u.pitch = 0;
    u.thrust = 0.1;
    u.mode = Controller_Output_t::VERT_THRU;
    u.yaw_mode = Controller_Output_t::CTRL_YAW_RATE;
    u.yaw = 0;
    int count_down = std::ceil(param.wait_before_take_off - sec_since_motor_armed());
    ROS_INFO("[n3ctrl] countdown to take off, %d...", count_down);
    return;
  }

  controller_last_active = ros::Time::now();

  double yaw_curr = get_yaw_from_quaternion(odom.q);
  Matrix3d wRc = rotz(yaw_curr); // intermediate frame or control frame, where control gains are defined

  Vector3d cmd_acc = command_acceleration_n3ctrl(des,odom);
  //Vector3d cmd_acc = command_acceleration_sertac(des,odom);

  Vector3d specific_thrust = cmd_acc - vg;
  // or use INDI
  //Vector3d specific_thrust = acceleration_loop(cmd_acc);

  Vector3d thrust_vector = regulate_trust_vector(specific_thrust * param.mass);

  // update thrust lpf here
  dbg_msg.lpf_thrust = to_vector3_msg(lpf_thrust.value);
  lpf_thrust.filter(thrust_vector/param.mass);

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
  dbg_msg.plant_a = to_vector3_msg(imu.q*imu.a);

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
      dbg_msg.lpf_acc = to_vector3_msg(lpf_acc.value);
      pub_dbg_info.publish(dbg_msg);
    }

  geometry_msgs::Vector3Stamped msg_disturbance;
  msg_disturbance.header.stamp = ros::Time::now();
  msg_disturbance.header.frame_id = "world";
  msg_disturbance.vector = to_vector3_msg(external_force_estimate());
  pub_disturbance.publish(msg_disturbance);

}

Eigen::Vector3d Controller::regulate_trust_vector(Eigen::Vector3d const & cmd ){

  Vector3d F_des = cmd;

  std::string constraint_info;

  if (F_des(2) < 0.5 * param.mass * param.gra){
    constraint_info = boost::str(boost::format("thrust too low F_des(2)=%.3f; ")% F_des(2));
    F_des = F_des / F_des(2) * (0.5 * param.mass * param.gra);
  }
  else if (F_des(2) > 5 * param.mass * param.gra){
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

Eigen::Vector3d Controller::acceleration_loop( Eigen::Vector3d const & cmd_acc ){
  return lpf_thrust.value + Kap * ( cmd_acc - lpf_acc.value );
}

void Controller::on_flight_status( std_msgs::UInt8ConstPtr const & msg ){
  if ( flight_status == flight_status_e::STOPED && msg->data > 0 ) {
    arm_time = ros::Time::now();
  }
  flight_status = (flight_status_e)msg->data;
}

bool Controller::can_disarm(const Odom_Data_t &odom) const {
  return odom.p.z() <= param.disarm.max_height;
}

bool Controller::is_disarm_cmd(const Desired_State_t &des) const {
  return des.p.z() < -0.1;
}

bool Controller::should_disarm(const Desired_State_t &des, const Odom_Data_t &odom ) const {
  return param.disarm.enable && can_disarm(odom) && is_disarm_cmd(des);
}

Eigen::Vector3d Controller::external_force_estimate(){
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d Controller::f_ext_indi() {
  return param.mass * ( lpf_acc.value - lpf_thrust.value - Vector3d::UnitZ() * param.gra );
}

Eigen::Vector3d Controller::command_acceleration_n3ctrl(const Desired_State_t &des, const Odom_Data_t &odom) {

  double yaw_curr = get_yaw_from_quaternion(odom.q);
  Matrix3d wRc = rotz(yaw_curr); // intermediate frame or control frame, where control gains are defined
  Matrix3d cRw = wRc.transpose();

  Vector3d e_p = des.p - odom.p;
  Vector3d cmd_vel = des.v + wRc * Kp * cRw * e_p; // P Controller

  dbg_msg.cmd_v = to_vector3_msg(cmd_vel);

  Vector3d e_v = cmd_vel - odom.v;
  Vector3d e_a = des.a - lpf_acc.value;

  Eigen::Vector3d u_v_i = vel_err_integral.update(e_v).output(Kvi,cRw);
  Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
  Eigen::Vector3d u_v_d = wRc * Ka * cRw * e_a;
  Eigen::Vector3d u_v = u_v_p + u_v_i + u_v_d; // PID Controller

  return u_v + des.a;

}

Eigen::Vector3d Controller::command_acceleration_sertac(const Desired_State_t &des, const Odom_Data_t &odom) {

  double yaw_curr = get_yaw_from_quaternion(odom.q);
  Matrix3d wRc = rotz(yaw_curr); // intermediate frame or control frame, where control gains are defined
  Matrix3d cRw = wRc.transpose();

  Vector3d e_p = des.p - odom.p;
  Vector3d e_v = des.v - odom.v;
  Vector3d e_a = des.a - lpf_acc.value;

  dbg_msg.err_p = to_vector3_msg(cRw*e_p);
  dbg_msg.err_v = to_vector3_msg(cRw*e_v);
  dbg_msg.err_a = to_vector3_msg(cRw*e_a);

  return   wRc * Kp * cRw * e_p
         + wRc * Kv * cRw * e_v
         + wRc * Ka * cRw * e_a;

}

void Controller::disarm(){
  ROS_WARN("[n3ctrl] sending disarm to sdk!!!");
  djiros::DroneArmControl arm;
  arm.request.arm = 0;
  ros::service::call("/djiros/drone_arm_control",arm);
}

double Controller::sec_since_motor_armed() const {
  if ( flight_status == flight_status_e::STOPED ) {
    return -1;
  }
  else {
    return (ros::Time::now() - arm_time).toSec();
  }
}