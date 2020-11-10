package n3ctrl;

public interface ControllerDebug extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "n3ctrl/ControllerDebug";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Vector3 des_p\ngeometry_msgs/Vector3 u_p_p\ngeometry_msgs/Vector3 u_p_i\ngeometry_msgs/Vector3 u_p\ngeometry_msgs/Vector3 des_v\ngeometry_msgs/Vector3 u_v_p\ngeometry_msgs/Vector3 u_v_i\ngeometry_msgs/Vector3 u_v\ngeometry_msgs/Vector3 k_p_p\ngeometry_msgs/Vector3 k_p_i\ngeometry_msgs/Vector3 k_v_p\ngeometry_msgs/Vector3 k_v_i\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getDesP();
  void setDesP(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getUPP();
  void setUPP(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getUPI();
  void setUPI(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getUP();
  void setUP(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getDesV();
  void setDesV(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getUVP();
  void setUVP(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getUVI();
  void setUVI(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getUV();
  void setUV(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getKPP();
  void setKPP(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getKPI();
  void setKPI(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getKVP();
  void setKVP(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getKVI();
  void setKVI(geometry_msgs.Vector3 value);
}
