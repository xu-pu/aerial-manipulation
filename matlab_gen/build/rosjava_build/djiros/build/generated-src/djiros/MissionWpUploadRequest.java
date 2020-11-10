package djiros;

public interface MissionWpUploadRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWpUploadRequest";
  static final java.lang.String _DEFINITION = "MissionWaypointTask waypoint_task   # see msg/MissionWaypointTask.msg\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  djiros.MissionWaypointTask getWaypointTask();
  void setWaypointTask(djiros.MissionWaypointTask value);
}
