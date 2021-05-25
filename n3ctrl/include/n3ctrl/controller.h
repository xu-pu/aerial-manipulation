#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <n3ctrl/ControllerDebug.h>
#include "input.h"

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	double yaw;
	Eigen::Quaterniond q;
	Eigen::Vector3d a;
};

struct Controller_Output_t
{
	static constexpr double VERT_VELO = -1.0;
	static constexpr double VERT_THRU = 1.0;

	// static constexpr double CTRL_YAW_RATE = -1.0; //zxzxzxzx
 //    static constexpr double CTRL_YAW = 1.0;
    static constexpr double CTRL_YAW_RATE = 1.0;
    static constexpr double CTRL_YAW = 0.0;

	double roll;
	double pitch;
	double yaw;
	double thrust;
	double mode; // if mode > 0, thrust = 0~100%;
				// if mode < 0, thrust = -? m/s ~ +? m/s
	double yaw_mode; // if yaw_mode > 0, CTRL_YAW;
				// if yaw_mode < 0, CTRL_YAW_RATE
};

struct SO3_Controller_Output_t
{
	Eigen::Matrix3d Rdes;
	Eigen::Vector3d Fdes;
	double net_force;
};

class Controller
{
public:
	Parameter_t& param;

	ros::Publisher ctrl_pub;
	ros::Publisher pub_dbg_info;

	n3ctrl::ControllerDebug dbg_msg;

	Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
	Eigen::Matrix3d Kap;
	double Kyaw;

	Eigen::Vector3d int_e_v;

	Controller(Parameter_t&);
	void config_gain(const Parameter_t::Gain& gain);
	void config();
	void update(const Desired_State_t& des, const Odom_Data_t& odom, const Imu_Data_t& imu, Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	
	void publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp, const ros::Time& extra_stamp);

private:
	bool is_configured;

	// XU PU
public:

    /**
     * Limit attitude angles and magnitude of the trust vector
     * @param cmd - thrust vector
     * @return regulated trust vector
     */
    Eigen::Vector3d regulate_cmd_thrust(Eigen::Vector3d const & cmd );

    /**
     * @param cmd_vel
     * @param des
     * @param odom
     * @return command acceleration - input to acceleration loop
     */
    Eigen::Vector3d velocity_loop( Eigen::Vector3d const & cmd_vel, const Desired_State_t& des,const Odom_Data_t& odom );

    /**
     * @param des
     * @param odom
     * @return command velocity - input to velocity loop
     */
    Eigen::Vector3d position_loop( const Desired_State_t& des,const Odom_Data_t& odom );

    Eigen::Vector3d acceleration_loop( Eigen::Vector3d const & F_des, const Imu_Data_t& imu, const Odom_Data_t& odom );

};

#endif
