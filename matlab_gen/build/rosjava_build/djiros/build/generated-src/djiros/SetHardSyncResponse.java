package djiros;

public interface SetHardSyncResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/SetHardSyncResponse";
  static final java.lang.String _DEFINITION = "uint8 result";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getResult();
  void setResult(byte value);
}
