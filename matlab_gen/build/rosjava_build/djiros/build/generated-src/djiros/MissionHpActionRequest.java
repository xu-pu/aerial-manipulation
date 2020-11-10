package djiros;

public interface MissionHpActionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpActionRequest";
  static final java.lang.String _DEFINITION = "#constant for action\nuint8 ACTION_START  = 0\nuint8 ACTION_STOP   = 1\nuint8 ACTION_PAUSE  = 2\nuint8 ACTION_RESUME = 3\n\n#response\nuint8 action          # see constants above for possible actions\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte ACTION_START = 0;
  static final byte ACTION_STOP = 1;
  static final byte ACTION_PAUSE = 2;
  static final byte ACTION_RESUME = 3;
  byte getAction();
  void setAction(byte value);
}
