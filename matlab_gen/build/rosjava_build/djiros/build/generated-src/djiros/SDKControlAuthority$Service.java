package djiros;

public interface SDKControlAuthority$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/SDKControlAuthority$Service";
  static final java.lang.String _DEFINITION = "#constant for control_enable\nuint8 RELEASE_CONTROL = 0\nuint8 REQUEST_CONTROL = 1\n\n#request\nuint8 control_enable    # see constants above\n---\nbool result\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
