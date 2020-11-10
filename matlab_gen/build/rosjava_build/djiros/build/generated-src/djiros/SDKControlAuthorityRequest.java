package djiros;

public interface SDKControlAuthorityRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/SDKControlAuthorityRequest";
  static final java.lang.String _DEFINITION = "#constant for control_enable\nuint8 RELEASE_CONTROL = 0\nuint8 REQUEST_CONTROL = 1\n\n#request\nuint8 control_enable    # see constants above\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte RELEASE_CONTROL = 0;
  static final byte REQUEST_CONTROL = 1;
  byte getControlEnable();
  void setControlEnable(byte value);
}
