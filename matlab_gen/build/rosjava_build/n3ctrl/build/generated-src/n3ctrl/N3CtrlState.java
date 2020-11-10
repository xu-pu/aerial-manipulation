package n3ctrl;

public interface N3CtrlState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "n3ctrl/N3CtrlState";
  static final java.lang.String _DEFINITION = "Header header\n\nuint8 STATE_DIRECT_CTRL = 0\nuint8 STATE_JS_CTRL = 1\nuint8 STATE_JS_NO_CTRL = 2\nuint8 STATE_JS_RESET_POS_CTRL = 3\nuint8 STATE_CMD_HOVER = 4\nuint8 STATE_CMD_CTRL = 5\nuint8 STATE_CMD_NO_CTRL = 6\nuint8 STATE_CMD_RESET_POS_CTRL = 7\nuint8 state\n\nuint32 last_traj_id";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte STATE_DIRECT_CTRL = 0;
  static final byte STATE_JS_CTRL = 1;
  static final byte STATE_JS_NO_CTRL = 2;
  static final byte STATE_JS_RESET_POS_CTRL = 3;
  static final byte STATE_CMD_HOVER = 4;
  static final byte STATE_CMD_CTRL = 5;
  static final byte STATE_CMD_NO_CTRL = 6;
  static final byte STATE_CMD_RESET_POS_CTRL = 7;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getState();
  void setState(byte value);
  int getLastTrajId();
  void setLastTrajId(int value);
}
