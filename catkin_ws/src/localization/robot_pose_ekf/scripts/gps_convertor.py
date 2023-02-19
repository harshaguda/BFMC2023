#!/usr/bin/env python3


import numpy as np
import os
import rospy
from utils.msg import localisation as oldGps
from nav_msgs.msg import Odometry as newGps
from tf.transformations import quaternion_from_euler
import time

class GpsConver():
    def __init__(self):
        rospy.init_node('GpsConvertNode', anonymous=False)
        self.oldGpsSub = rospy.Subscriber("/automobile/localisation", oldGps, self.callback)
        self.newGpsPub = rospy.Publisher('/localization/gps_converted', newGps, queue_size=1)
        g = 1e-6
        b = 1e6
        self.twist_covariance = np.diag([b, b, b, b, b, b]).flatten()
        self.pos_covariance = np.diag([g, g, g, b, b, g]).flatten()
        rospy.spin()

    def callback(self, data):
        msg_to_send = newGps()
        # header
        msg_to_send.header.stamp = rospy.get_rostime()
        msg_to_send.header.frame_id = "gps_link"
        # Pos
            # position
        msg_to_send.pose.pose.position.x, msg_to_send.pose.pose.position.y, msg_to_send.pose.pose.position.z \
            = data.posA, data.posB, 0
            # orientation
        q = quaternion_from_euler(0, 0, data.rotA)
        msg_to_send.pose.pose.orientation.x, msg_to_send.pose.pose.orientation.y, msg_to_send.pose.pose.orientation.z, msg_to_send.pose.pose.orientation.w \
            = q[0], q[1], q[2], q[3]
        msg_to_send.pose.covariance = self.pos_covariance
        # Twist
        msg_to_send.twist.covariance = self.twist_covariance
        # pub
        self.newGpsPub.publish(msg_to_send)
        

if __name__ == '__main__':
    try:
        nod = GpsConver()
    except rospy.ROSInterruptException:
        pass