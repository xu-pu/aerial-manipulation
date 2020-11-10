package djiros;

public interface MissionWaypoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWaypoint";
  static final java.lang.String _DEFINITION = "float64 latitude          # radian\nfloat64 longitude         # radian\nfloat32 altitude          # relative altitude from takeoff point\nfloat32 damping_distance  # Bend length (effective coordinated turn mode only)\nint16 target_yaw          # Yaw (degree)\nint16 target_gimbal_pitch # Gimbal pitch\nuint8 turn_mode           # 0: clockwise, 1: counter-clockwise\nuint8 has_action          # 0: no, 1: yes\nuint16 action_time_limit\nMissionWaypointAction waypoint_action\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getLatitude();
  void setLatitude(double value);
  double getLongitude();
  void setLongitude(double value);
  float getAltitude();
  void setAltitude(float value);
  float getDampingDistance();
  void setDampingDistance(float value);
  short getTargetYaw();
  void setTargetYaw(short value);
  short getTargetGimbalPitch();
  void setTargetGimbalPitch(short value);
  byte getTurnMode();
  void setTurnMode(byte value);
  byte getHasAction();
  void setHasAction(byte value);
  short getActionTimeLimit();
  void setActionTimeLimit(short value);
  djiros.MissionWaypointAction getWaypointAction();
  void setWaypointAction(djiros.MissionWaypointAction value);
}
