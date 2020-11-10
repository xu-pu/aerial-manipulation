package djiros;

public interface MissionStatusResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionStatusResponse";
  static final java.lang.String _DEFINITION = "uint8 waypoint_mission_count\nuint8 hotpoint_mission_count";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getWaypointMissionCount();
  void setWaypointMissionCount(byte value);
  byte getHotpointMissionCount();
  void setHotpointMissionCount(byte value);
}
