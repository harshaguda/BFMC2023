#!/usr/bin/env python3


import numpy as np
import os
import rospy
from utils.msg import IMU as oldImu
from sensor_msgs.msg import Imu as newImu
from tf.transformations import quaternion_from_euler
import time

class ImuConver():
    def __init__(self):
        rospy.init_node('ImuConvertNode', anonymous=False)
        self.oldImuSub = rospy.Subscriber("/automobile/IMU", oldImu, self.callback)
        self.newImuPub = rospy.Publisher('/localization/imu_converted', newImu, queue_size=1)
        g = 1e-6
        b = 1e6
        # self.orientation_cov = np.diag([g, g, g]).flatten()
        self.orientation_cov = np.diag([b, b, b]).flatten()
        self.angular_velocity_cov = np.diag([b, b, b]).flatten()
        self.linear_velocity_cov = np.diag([g, g, g]).flatten()
        # self.linear_velocity_cov = np.diag([b, b, b]).flatten()
        rospy.spin()

    def callback(self, data):
        msg_to_send = newImu()
        # header
        msg_to_send.header.stamp = rospy.get_rostime()
        msg_to_send.header.frame_id = "imu_link"
        # orientation
        q = quaternion_from_euler(data.roll, data.pitch, data.yaw)
        msg_to_send.orientation.x, msg_to_send.orientation.y, msg_to_send.orientation.z, msg_to_send.orientation.w \
            = q[0], q[1], q[2], q[3]
        msg_to_send.orientation_covariance = self.orientation_cov
        # angular velocity
        msg_to_send.angular_velocity_covariance = self.angular_velocity_cov
        # linear velocity
        msg_to_send.linear_acceleration.x, msg_to_send.linear_acceleration.y, msg_to_send.linear_acceleration.z \
            = data.accelx, data.accely, data.accelz
        msg_to_send.linear_acceleration_covariance = self.linear_velocity_cov
        # pub
        self.newImuPub.publish(msg_to_send)
        

if __name__ == '__main__':
    try:
        nod = ImuConver()
    except rospy.ROSInterruptException:
        pass