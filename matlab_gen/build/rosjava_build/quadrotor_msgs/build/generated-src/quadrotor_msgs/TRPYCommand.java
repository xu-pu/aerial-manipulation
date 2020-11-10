package quadrotor_msgs;

public interface TRPYCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/TRPYCommand";
  static final java.lang.String _DEFINITION = "Header header\nfloat32 thrust\nfloat32 roll\nfloat32 pitch\nfloat32 yaw\nquadrotor_msgs/AuxCommand aux\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getThrust();
  void setThrust(float value);
  float getRoll();
  void setRoll(float value);
  float getPitch();
  void setPitch(float value);
  float getYaw();
  void setYaw(float value);
  quadrotor_msgs.AuxCommand getAux();
  void setAux(quadrotor_msgs.AuxCommand value);
}
