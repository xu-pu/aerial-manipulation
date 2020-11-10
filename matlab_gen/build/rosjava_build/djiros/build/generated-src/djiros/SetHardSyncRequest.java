package djiros;

public interface SetHardSyncRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/SetHardSyncRequest";
  static final java.lang.String _DEFINITION = "uint32 frequency  # frequency in Hz\nuint16 tag        # the tag is to distinguish between different call\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  int getFrequency();
  void setFrequency(int value);
  short getTag();
  void setTag(short value);
}
