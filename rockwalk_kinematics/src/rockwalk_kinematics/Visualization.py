
import rospy
import math
import tf
import numpy as np

from tf import transformations as tfms

from geometry_msgs.msg import Point, Quaternion, Pose, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA


OBJECT_RADIUS = 0.35
# FROM BODY FRAME TO MARKER FRAME
BODY_MARKERFRAME_QUATERNION = tfms.quaternion_from_euler(math.pi/2, math.pi/2, 0, 'rxyz')

BODY_MARKERFRAME_POSITION = np.array([[-OBJECT_RADIUS],[-OBJECT_RADIUS],[0],[0]])


class KinematicsVisualization:

    def __init__(self, pub_kinematics, object_kinematics):

        self._pub_kinematics = pub_kinematics
        self._object_kinematics = object_kinematics


    def publish_object_marker(self):

        world_markerframe_quaternion = tfms.quaternion_multiply(self._object_kinematics._world_bodyframe_quaternion,
                                                                BODY_MARKERFRAME_QUATERNION)

        world_bodyframe_rot = tfms.quaternion_matrix(self._object_kinematics._world_bodyframe_quaternion)
        world_diskcenter_marker_vec = np.matmul(world_bodyframe_rot, BODY_MARKERFRAME_POSITION)

        marker_pose = Pose()
        marker_pose.position = Point(self._object_kinematics._disk_center_position.x + world_diskcenter_marker_vec[0],
                                     self._object_kinematics._disk_center_position.y + world_diskcenter_marker_vec[1],
                                     self._object_kinematics._disk_center_position.z + world_diskcenter_marker_vec[2])

        marker_pose.orientation = Quaternion(world_markerframe_quaternion[0],
                                             world_markerframe_quaternion[1],
                                             world_markerframe_quaternion[2],
                                             world_markerframe_quaternion[3])

        marker = Marker(
                    type=Marker.MESH_RESOURCE,
                    action=Marker.ADD,
                    id=0,
                    pose = marker_pose,
                    scale=Vector3(0.001, .001, .001),
                    header=Header(frame_id="world"),
                    color=ColorRGBA(.70,.70,.70 ,1),
                    mesh_resource="package://rockwalk_kinematics/object_model/rockwalk_cone.dae"
                    )

        self._pub_kinematics._object_marker_publisher.publish(marker)
