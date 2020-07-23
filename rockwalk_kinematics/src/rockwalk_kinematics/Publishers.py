import rospy

from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped
from visualization_msgs.msg import Marker



class KinematicsPublishers:

    def __init__(self):
        self.initialize_publishers()

    def initialize_publishers(self):
        rospy.loginfo("Initializing the following topics:")
        rospy.loginfo("1. Body euler @body_euler")
        rospy.loginfo("2. Body twist @body_twist")
        rospy.loginfo("3. Object pose @object_pose")
        rospy.loginfo("4. Ground contact @ground_contact")

        self._euler_publisher = rospy.Publisher('body_euler', Vector3, queue_size=10)
        self._twist_publisher = rospy.Publisher('body_twist', TwistStamped, queue_size=10)
        self._object_pose_publisher = rospy.Publisher('object_pose', PoseStamped, queue_size=10)
        self._ground_contact_publisher = rospy.Publisher('ground_contact', Vector3, queue_size=10)

        self._object_marker_publisher = rospy.Publisher('object_marker', Marker, queue_size=10)
