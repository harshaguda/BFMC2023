#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
import sys

class CameraHandler():
    # ===================================== INIT==========================================
    def __init__(self, data_path, process_id):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.data_path = data_path
        self.process_id =process_id
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        timestr = time.strftime("%Y%m%d-%H%M%S")
        img_name = timestr + ".jpg"
        img_dir = os.path.join(self.data_path, img_name)
        cv2.imwrite(img_dir, self.cv_image)
        print("image {0} saved to {1}".format(img_name, self.data_path))
        cmd = "kill -9 {0}".format(self.process_id)
        os.system(cmd)
    

if __name__ == '__main__':
    # try:
    #     data_type = sys.argv[1]
    # except:
    #     exit("Error: please input either sign light or ped as command line argument")
    # if data_type=="sign":
    #     data_path = "data/simul/sign"
    # elif data_type=="light":
    #     data_path = "data/simul/light"
    # elif data_type=="ped":
    #     data_path = "data/simul/ped"
    # else:
    #     exit("Error: please input either sign light or ped as command line argument")
    data_path = "data/simul/validation"
    try:
        rl_path = os.path.realpath(os.path.dirname(__file__))
        data_rl_path = os.path.join(rl_path, data_path)
        process_id = os.getpid()
        nod = CameraHandler(data_rl_path, process_id)
    except rospy.ROSInterruptException:
        pass
