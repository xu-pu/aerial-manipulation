package djiros;

public interface MissionWpGetInfoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWpGetInfoResponse";
  static final java.lang.String _DEFINITION = "MissionWaypointTask waypoint_task";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  djiros.MissionWaypointTask getWaypointTask();
  void setWaypointTask(djiros.MissionWaypointTask value);
}
