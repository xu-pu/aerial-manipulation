package djiros;

public interface DroneArmControlRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/DroneArmControlRequest";
  static final java.lang.String _DEFINITION = "#constant for arm\nuint8 DISARM_COMMAND = 0\nuint8 ARM_COMMAND    = 1\n\n#request\nuint8 arm    # see constants\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte DISARM_COMMAND = 0;
  static final byte ARM_COMMAND = 1;
  byte getArm();
  void setArm(byte value);
}
