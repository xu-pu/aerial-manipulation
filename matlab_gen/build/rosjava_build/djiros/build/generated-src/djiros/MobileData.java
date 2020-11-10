package djiros;

public interface MobileData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/MobileData";
  static final java.lang.String _DEFINITION = "uint8[] data";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}
