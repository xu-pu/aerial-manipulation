/*
Upload this code to Arduino Mega affixed to 9-axis motion shield.
This will help publish motion shield twist and its orientation to ROS
*/

#include <ros.h>
#include <ros/time.h>
#include "NineAxesMotion.h"
#include <Wire.h>
// import ros messages
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>


NineAxesMotion mySensor;  //Object that for the sensor

// ros instantiation
ros::NodeHandle nh;


geometry_msgs::QuaternionStamped quat_msg;
geometry_msgs::TwistStamped twist_msg;

ros::Publisher twist("twist_motion_shield", &twist_msg);
ros::Publisher quat("quat_motion_shield", &quat_msg);

void setup() {
  nh.getHardware()->setBaud(115200);

  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.

  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF_FMC_OFF);
  mySensor.updateAccelConfig();

  nh.initNode();
  nh.advertise(twist);
  nh.advertise(quat);

}

void loop() {

    quat_msg.quaternion.x = mySensor.readQuaternion(X_QUAT);
    quat_msg.quaternion.y = mySensor.readQuaternion(Y_QUAT);
    quat_msg.quaternion.z = mySensor.readQuaternion(Z_QUAT);
    quat_msg.quaternion.w = mySensor.readQuaternion(W_QUAT);

    twist_msg.twist.angular.x = mySensor.readGyro(X_AXIS);
    twist_msg.twist.angular.y = mySensor.readGyro(Y_AXIS);
    twist_msg.twist.angular.z = mySensor.readGyro(Z_AXIS);

    quat.publish(&quat_msg);
    twist.publish(&twist_msg);

    nh.spinOnce();
    delay(0.01);
}
