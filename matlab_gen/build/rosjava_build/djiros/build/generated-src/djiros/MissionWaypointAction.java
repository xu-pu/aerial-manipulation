package djiros;

public interface MissionWaypointAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MissionWaypointAction";
  static final java.lang.String _DEFINITION = "# first 4 bit: Total number of actions\n# last 4 bit: Total running times\nuint8 action_repeat\nuint8[16] command_list\nint16[16] command_parameter\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  byte getActionRepeat();
  void setActionRepeat(byte value);
  org.jboss.netty.buffer.ChannelBuffer getCommandList();
  void setCommandList(org.jboss.netty.buffer.ChannelBuffer value);
  short[] getCommandParameter();
  void setCommandParameter(short[] value);
}
