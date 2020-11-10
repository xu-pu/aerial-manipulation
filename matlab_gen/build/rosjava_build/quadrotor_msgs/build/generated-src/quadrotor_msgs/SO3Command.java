package quadrotor_msgs;

public interface SO3Command extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/SO3Command";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Vector3 force\ngeometry_msgs/Quaternion orientation\nfloat64[3] kR\nfloat64[3] kOm\nquadrotor_msgs/AuxCommand aux\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getForce();
  void setForce(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double[] getKR();
  void setKR(double[] value);
  double[] getKOm();
  void setKOm(double[] value);
  quadrotor_msgs.AuxCommand getAux();
  void setAux(quadrotor_msgs.AuxCommand value);
}
