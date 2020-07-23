#!/usr/bin/env python

import rospy

from rockwalk_kinematics.Subscribers import SubscriberMotionShield
from rockwalk_kinematics.Publishers import KinematicsPublishers
from rockwalk_kinematics.RollingKinematics import Kinematics
from rockwalk_kinematics.Visualization import KinematicsVisualization

if __name__ == '__main__':
    rospy.init_node("rockwalk_kinematics_node", anonymous=True)


    sub_motion_shield = SubscriberMotionShield()

    pub_kinematics = KinematicsPublishers()
    rospy.sleep(3)

    object_kinematics = Kinematics(sub_motion_shield, pub_kinematics)

    object_visualization = KinematicsVisualization(pub_kinematics,object_kinematics)


    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        object_kinematics.compute_body_euler()
        object_kinematics.compute_body_twist()
        object_kinematics.compute_object_pose()
        object_kinematics.compute_contact_coordinates()

        object_visualization.publish_object_marker()

        rate.sleep()
    rospy.spin()
