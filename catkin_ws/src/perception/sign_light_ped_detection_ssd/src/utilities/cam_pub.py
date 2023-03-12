#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from picamera2 import Picamera2

class CameraPub():
    def __init__(self):
        rospy.init_node('Campubnode', anonymous=False)     
        self.publisher = rospy.Publisher("/automobile/image_raw", Image, queue_size=1)
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2.start()

    def run(self):
        while True:
            image = self.picam2.capture_array()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            imageObject = CvBridge().cv2_to_imgmsg(image, "bgr8")
            imageObject.header.stamp = rospy.Time.now()
            self.publisher.publish(imageObject)
        

if __name__ == '__main__':
    try:
        nod = CameraPub()
        nod.run()
    except rospy.ROSInterruptException:
        pass
