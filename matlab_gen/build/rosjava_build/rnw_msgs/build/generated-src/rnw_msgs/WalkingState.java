package rnw_msgs;

public interface WalkingState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rnw_msgs/WalkingState";
  static final java.lang.String _DEFINITION = "Header header\nuint32 step_count\nrnw_msgs/ConeState cone_state\nfloat64 tau_deg\nfloat64 desired_nutation_deg";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getStepCount();
  void setStepCount(int value);
  rnw_msgs.ConeState getConeState();
  void setConeState(rnw_msgs.ConeState value);
  double getTauDeg();
  void setTauDeg(double value);
  double getDesiredNutationDeg();
  void setDesiredNutationDeg(double value);
}
