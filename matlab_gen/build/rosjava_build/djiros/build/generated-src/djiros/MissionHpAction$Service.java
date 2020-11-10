package djiros;

public interface MissionHpAction$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionHpAction$Service";
  static final java.lang.String _DEFINITION = "#constant for action\nuint8 ACTION_START  = 0\nuint8 ACTION_STOP   = 1\nuint8 ACTION_PAUSE  = 2\nuint8 ACTION_RESUME = 3\n\n#response\nuint8 action          # see constants above for possible actions\n---\nbool result\n# for debugging usage\nuint8 cmd_set\nuint8 cmd_id\nuint32 ack_data\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
