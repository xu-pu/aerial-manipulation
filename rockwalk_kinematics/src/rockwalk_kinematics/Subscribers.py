import rospy
import copy
from geometry_msgs.msg import QuaternionStamped, TwistStamped

from tf import transformations as tfms


class SubscriberMotionShield:
    """Subscriber to QuaternionStamped and TwistStamped topics
       from 9-axis arduino motion shield
    """

    def __init__(self):

        rospy.loginfo("Subscribing to Quaternion Topic from Motion Shield")
        self._quaternion_sub =rospy.Subscriber("quat_motion_shield", QuaternionStamped, self.store_quaternion_data)

        rospy.loginfo("Subscribing to Twist Topic from Motion Shield")
        self._twist_sub =rospy.Subscriber("twist_motion_shield", TwistStamped, self.store_twist_data)

    def store_quaternion_data(self, quaternion_data):

        self._imu_quaternion = tfms.unit_vector([quaternion_data.quaternion.x/1000.0,
                                                 quaternion_data.quaternion.y/1000.0,
                                                 quaternion_data.quaternion.z/1000.0,
                                                 quaternion_data.quaternion.w/1000.0])


    def store_twist_data(self, twist_data):
        self._imu_twist = copy.deepcopy(twist_data) #in deg/s
