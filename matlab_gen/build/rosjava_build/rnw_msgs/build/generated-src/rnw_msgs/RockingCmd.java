package rnw_msgs;

public interface RockingCmd extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rnw_msgs/RockingCmd";
  static final java.lang.String _DEFINITION = "Header header\nuint8 fsm\nuint8 cmd_type\nuint32 cmd_idx\n# waypoint\ngeometry_msgs/Point setpoint_uav\ngeometry_msgs/Point setpoint_apex\n# walking\nbool is_walking\nuint8 walk_idx\nfloat64 tau_deg\nuint32 step_count\n# grip monitoring\nrnw_msgs/GripState grip_state\nfloat64 setpoint_nutation_deg\nfloat64 setpoint_grip_depth\nfloat64 err_grip_depth\nfloat64 err_nutation_deg\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getFsm();
  void setFsm(byte value);
  byte getCmdType();
  void setCmdType(byte value);
  int getCmdIdx();
  void setCmdIdx(int value);
  geometry_msgs.Point getSetpointUav();
  void setSetpointUav(geometry_msgs.Point value);
  geometry_msgs.Point getSetpointApex();
  void setSetpointApex(geometry_msgs.Point value);
  boolean getIsWalking();
  void setIsWalking(boolean value);
  byte getWalkIdx();
  void setWalkIdx(byte value);
  double getTauDeg();
  void setTauDeg(double value);
  int getStepCount();
  void setStepCount(int value);
  rnw_msgs.GripState getGripState();
  void setGripState(rnw_msgs.GripState value);
  double getSetpointNutationDeg();
  void setSetpointNutationDeg(double value);
  double getSetpointGripDepth();
  void setSetpointGripDepth(double value);
  double getErrGripDepth();
  void setErrGripDepth(double value);
  double getErrNutationDeg();
  void setErrNutationDeg(double value);
}
