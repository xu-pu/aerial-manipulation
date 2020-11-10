package quadrotor_msgs;

public interface Float64Stamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/Float64Stamped";
  static final java.lang.String _DEFINITION = "Header header\nfloat64 value";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getValue();
  void setValue(double value);
}
