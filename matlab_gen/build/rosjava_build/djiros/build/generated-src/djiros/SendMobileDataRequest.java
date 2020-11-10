package djiros;

public interface SendMobileDataRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "djiros/SendMobileDataRequest";
  static final java.lang.String _DEFINITION = "uint8[] data  #length(data) <= 100\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  org.jboss.netty.buffer.ChannelBuffer getData();
  void setData(org.jboss.netty.buffer.ChannelBuffer value);
}
