package djiros;

public interface MissionWpUpload$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWpUpload$Service";
  static final java.lang.String _DEFINITION = "MissionWaypointTask waypoint_task   # see msg/MissionWaypointTask.msg\n---\nbool result\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
