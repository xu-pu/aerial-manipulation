package quadrotor_msgs;

public interface SwarmCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/SwarmCommand";
  static final java.lang.String _DEFINITION = "#data structure\nHeader        header\nint32[]       selection\nnav_msgs/Path plan\nfloat32[]     formation #todo implement related code\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int[] getSelection();
  void setSelection(int[] value);
  nav_msgs.Path getPlan();
  void setPlan(nav_msgs.Path value);
  float[] getFormation();
  void setFormation(float[] value);
}
