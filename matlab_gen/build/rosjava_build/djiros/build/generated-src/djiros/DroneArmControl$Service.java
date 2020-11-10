package djiros;

public interface DroneArmControl$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/DroneArmControl$Service";
  static final java.lang.String _DEFINITION = "#constant for arm\nuint8 DISARM_COMMAND = 0\nuint8 ARM_COMMAND    = 1\n\n#request\nuint8 arm    # see constants\n---\nbool result\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
