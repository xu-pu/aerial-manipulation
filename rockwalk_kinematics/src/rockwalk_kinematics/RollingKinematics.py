import rospy

import math
import numpy as np

from tf import transformations as tfms

from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3

import Utilities


OBJECT_RADIUS = 0.35 # Base radius of (cone)

# Check how you imu if fixed on the object relative to the body frame
# To see how body frame is defined take a look at the README.md file
IMU_BODYFRAME_QUATERNION = tfms.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'rxyz')
# Read as: FROM IMU TO BODY FRAME QUATERNION

# Check how the imu (on the object) is initially oriented relative to the world frame
IMU_WORLDFRAME_QUATERNION = tfms.quaternion_from_euler(math.pi/2, 0, math.pi/2, 'rxyz')
# The above two definitions mean that initially the body frame is aligned with the world frame.
#----

class Kinematics:

    def __init__(self, sub_motion_shield, pub_kinematics):

        self._sub_motion_shield = sub_motion_shield
        self._pub_kinematics = pub_kinematics


        global CURRENT_POSITION, CURRENT_TIME

        CURRENT_POSITION = Vector3(0,0,0) # Begin position of the object at 0,0,0
        CURRENT_TIME = rospy.get_time()

        self.record_initial_object_configuration()


    def record_initial_object_configuration(self):

        rospy.loginfo("Bring the object to its initial configuration:")
        rospy.sleep(2)
        self._earth_initial_configuration_quaternion = self._sub_motion_shield._imu_quaternion
        rospy.loginfo("Initial IMU quaternion recorded")


    def compute_body_euler(self):


        earth_worldframe_quaternion = tfms.quaternion_multiply(self._earth_initial_configuration_quaternion,
                                                               IMU_WORLDFRAME_QUATERNION)

        earth_bodyframe_quaternion = tfms.quaternion_multiply(self._sub_motion_shield._imu_quaternion,
                                                              IMU_BODYFRAME_QUATERNION)

        self._world_bodyframe_quaternion = tfms.quaternion_multiply(tfms.quaternion_inverse(earth_worldframe_quaternion),
                                                                    earth_bodyframe_quaternion)

        world_bodyframe_rot = tfms.quaternion_matrix(self._world_bodyframe_quaternion)

        [psi, theta, phi] = Utilities.compute_euler(world_bodyframe_rot)

        self._body_euler = Vector3(psi, theta, phi)

        #publish
        self._pub_kinematics._euler_publisher.publish(self._body_euler)



    def compute_body_twist(self):

        #-------------------------------
        #Angular velocity
        twist_vec = np.array([[np.radians(self._sub_motion_shield._imu_twist.twist.angular.x)],
                              [np.radians(self._sub_motion_shield._imu_twist.twist.angular.y)],
                              [np.radians(self._sub_motion_shield._imu_twist.twist.angular.z)],
                              [0]])

        imu_bodyframe_rot = tfms.quaternion_matrix(IMU_BODYFRAME_QUATERNION)
        angular_velocity_bodyframe = np.matmul(np.transpose(imu_bodyframe_rot),twist_vec)

        #-------------------------------
        #Translational Velocity from nonholonomic contact
        psi = self._body_euler.x
        theta = self._body_euler.y
        phi = self._body_euler.z

        psi_dot = angular_velocity_bodyframe[0]
        theta_dot = angular_velocity_bodyframe[1]
        phi_dot = angular_velocity_bodyframe[2]

        trans_vel_x = OBJECT_RADIUS*psi_dot*math.cos(psi)*math.cos(theta) - \
                      OBJECT_RADIUS*theta_dot*math.sin(psi)*math.sin(theta) + \
                      OBJECT_RADIUS*phi_dot*math.cos(psi)

        trans_vel_y = OBJECT_RADIUS*psi_dot*math.sin(psi)*math.cos(theta) - \
                      OBJECT_RADIUS*theta_dot*math.cos(psi)*math.sin(theta) + \
                      OBJECT_RADIUS*phi_dot*math.sin(psi)

        trans_vel_z = OBJECT_RADIUS*theta_dot*math.cos(theta)

        #-------------------------------


        self._body_twist = TwistStamped()
        self._body_twist.header.stamp = rospy.Time.now()
        self._body_twist.twist.angular = Vector3(psi_dot, theta_dot, phi_dot) # in rad/s
        self._body_twist.twist.linear = Vector3(trans_vel_x, trans_vel_y, trans_vel_z)

        #publish
        self._pub_kinematics._twist_publisher.publish(self._body_twist)


    def compute_object_pose(self):
        """
        Position: Base (disk) center coordinates (x_D, y_D, z_D)
        Orientation: Quaternion representing body frame relative to the world frame
        """
        global CURRENT_POSITION, CURRENT_TIME

        [CURRENT_POSITION, CURRENT_TIME] = self.integrate_linear_velocities(CURRENT_POSITION,
                                                                            CURRENT_TIME)

        self._object_pose = PoseStamped()
        self._object_pose.header.stamp = rospy.Time.now()
        self._object_pose.header.frame_id = "world"
        self._object_pose.pose.position = Point(self._disk_center_position.x,
                                                self._disk_center_position.y,
                                                self._disk_center_position.z)


        self._object_pose.pose.orientation = Quaternion(self._world_bodyframe_quaternion[0],
                                                        self._world_bodyframe_quaternion[1],
                                                        self._world_bodyframe_quaternion[2],
                                                        self._world_bodyframe_quaternion[3])


        #publish
        self._pub_kinematics._object_pose_publisher.publish(self._object_pose)


    def integrate_linear_velocities(self, current_position, current_time):
        """Base of the cone modeled as a circular disk"""

        new_time = self._body_twist.header.stamp.to_sec()

        position_change_x = self._body_twist.twist.linear.x * (new_time - current_time)
        position_change_y = self._body_twist.twist.linear.y * (new_time - current_time)

        self._disk_center_position = Vector3()
        self._disk_center_position.x = current_position.x + position_change_x
        self._disk_center_position.y = current_position.y + position_change_y
        self._disk_center_position.z = OBJECT_RADIUS*math.sin(self._body_euler.y)

        return self._disk_center_position, new_time



    def compute_contact_coordinates(self):
        """Trace of ground contact as the object rolls without slipping"""

        rot_psi = tfms.rotation_matrix(self._body_euler.x, [0,0,1])
        init_rot = tfms.rotation_matrix(math.pi/2, [0,0,1])
        rot_theta = tfms.rotation_matrix(self._body_euler.y, [0,1,0])
        rot_phi = tfms.rotation_matrix(self._body_euler.z, [0,0,1])


        rot_StoQ = np.matmul(np.matmul(rot_psi, init_rot),rot_theta) # this frame
        # does not roll with the object. So suitable to determine ground contact point

        rot_StoB = np.matmul(rot_StoQ, rot_phi) # from world to body frame. no use here.

        ground_contact_vector = np.matmul(rot_StoQ,np.array([[OBJECT_RADIUS],[0],[0],[0]]))

        contact_position_x = self._disk_center_position.x + ground_contact_vector[0]
        contact_position_y = self._disk_center_position.y + ground_contact_vector[1]

        self._ground_contact = Vector3()
        self._ground_contact.x = contact_position_x
        self._ground_contact.y = contact_position_y
        self._ground_contact.z = 0

        #publish
        self._pub_kinematics._ground_contact_publisher.publish(self._ground_contact)
