package quadrotor_msgs;

public interface OutputData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/OutputData";
  static final java.lang.String _DEFINITION = "Header header\nuint16 loop_rate\nfloat64 voltage\ngeometry_msgs/Quaternion orientation\ngeometry_msgs/Vector3 angular_velocity\ngeometry_msgs/Vector3 linear_acceleration\nfloat64 pressure_dheight\nfloat64 pressure_height\ngeometry_msgs/Vector3 magnetic_field\nuint8[8] radio_channel\n#uint8[4] motor_rpm\nuint8 seq\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  short getLoopRate();
  void setLoopRate(short value);
  double getVoltage();
  void setVoltage(double value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getLinearAcceleration();
  void setLinearAcceleration(geometry_msgs.Vector3 value);
  double getPressureDheight();
  void setPressureDheight(double value);
  double getPressureHeight();
  void setPressureHeight(double value);
  geometry_msgs.Vector3 getMagneticField();
  void setMagneticField(geometry_msgs.Vector3 value);
  org.jboss.netty.buffer.ChannelBuffer getRadioChannel();
  void setRadioChannel(org.jboss.netty.buffer.ChannelBuffer value);
  byte getSeq();
  void setSeq(byte value);
}
