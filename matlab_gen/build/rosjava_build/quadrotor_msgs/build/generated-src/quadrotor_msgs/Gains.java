package quadrotor_msgs;

public interface Gains extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Gains";
  static final java.lang.String _DEFINITION = "float64 Kp\nfloat64 Kd\nfloat64 Kp_yaw\nfloat64 Kd_yaw\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getKp();
  void setKp(double value);
  double getKd();
  void setKd(double value);
  double getKpYaw();
  void setKpYaw(double value);
  double getKdYaw();
  void setKdYaw(double value);
}
