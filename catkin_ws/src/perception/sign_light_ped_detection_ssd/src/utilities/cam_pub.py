#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraPub():
    def __init__(self):
        rospy.init_node('Campubnode', anonymous=False)     
        self.publisher = rospy.Publisher("/automobile/image_raw", Image, queue_size=1)
    def run(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
                print("Cannot open camera")
                exit()
        else:
                ret, _ = cap.read()
        while ret:
                ret, image = cap.read()
                imageObject = CvBridge().cv2_to_imgmsg(image, "bgr8")
                imageObject.header.stamp = rospy.Time.now()
                self.publisher.publish(imageObject)
        cap.release()

if __name__ == '__main__':
    try:
        nod = CameraPub()
        nod.run()
    except rospy.ROSInterruptException:
        pass
