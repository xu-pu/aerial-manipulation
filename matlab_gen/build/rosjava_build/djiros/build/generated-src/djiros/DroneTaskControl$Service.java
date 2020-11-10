package djiros;

public interface DroneTaskControl$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/DroneTaskControl$Service";
  static final java.lang.String _DEFINITION = "#constant for tasks\nuint8 TASK_GOHOME = 1\nuint8 TASK_TAKEOFF = 4\nuint8 TASK_LAND = 6\n\n#request\nuint8 task    # see constants above for possible tasks\n---\nbool result\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
