#!/usr/bin/env python3

from std_msgs.msg import String
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import os

class Datacollection():
    # ===================================== INIT==========================================
    def __init__(self, path):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.path = path
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        self.keys_sub = rospy.Subscriber('/pathcontrol/keys', String, self.keys_callback)
        rospy.spin()

    def image_callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
    def keys_callback(self, data):
        if data.data == "j":
            data_path = os.path.join(self.path, data.data)
        elif data.data == "l":
            data_path = os.path.join(self.path, data.data)
        elif (data.data == "i") or (data.data == "k"):
            data_path = os.path.join(self.path, "ik")
        rl_path = os.path.realpath(os.path.dirname(__file__))
        data_rl_path = os.path.join(rl_path, data_path)
        img_name = data.data + time.strftime("%Y%m%d-%H%M%S") + ".jpg"
        img_dir = os.path.join(data_rl_path, img_name)
        cv2.imwrite(img_dir, self.cv_image)
        print("image {0} saved to {1}".format(img_name, data_rl_path))

if __name__ == '__main__':
    try:
        path = "../data/track/train"
        nod = Datacollection(path)
    except rospy.ROSInterruptException:
        pass
