package djiros;

public interface DroneTaskControlRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/DroneTaskControlRequest";
  static final java.lang.String _DEFINITION = "#constant for tasks\nuint8 TASK_GOHOME = 1\nuint8 TASK_TAKEOFF = 4\nuint8 TASK_LAND = 6\n\n#request\nuint8 task    # see constants above for possible tasks\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte TASK_GOHOME = 1;
  static final byte TASK_TAKEOFF = 4;
  static final byte TASK_LAND = 6;
  byte getTask();
  void setTask(byte value);
}
