#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from std_msgs.msg      import String, Int16
import time
import json

from LaneDetFollow import perspective_warp

def pipeline(img, s_thresh=(100, 255), sx_thresh=(15, 255)):
    # img = undistort(img)
    img = np.copy(img)
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS).astype(np.float)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    h_channel = hls[:,:,0]
    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 0, 5, ksize=7, scale=1) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    
    color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255
    
    combined_binary = np.zeros_like(sxbinary)
    combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
    return combined_binary

def detect_stop_line(img):
    return 

class LaneDetection():
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('LanesDetectionNode', anonymous=False)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.stop_signnum = 0
        self.start = rospy.Time.now()
        self.stopcounter = rospy.Publisher('/lanes/stop_line', Int16, queue_size=1)
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        combined_binary = pipeline(self.cv_image)
        initp = np.float32([(0,480), (150, 280), (550, 280), (640, 480)])
        dst_ = np.float32([(0, 480), (0,0), (640,0), (640,480)])

        img = perspective_warp(combined_binary, src=initp, dst=dst_)
        # print(np.count_nonzero(current_frame))
        out_img = np.dstack((img, img, img))*255

        cv2.imshow('result', out_img[380:450,:])
        # print(np.count_nonzero(out_img[380:450,:]))
        nzcnt = np.count_nonzero(out_img[380:450,:])
        if nzcnt > 10000:
            if rospy.Time.now() - self.start > rospy.Duration(3):
                self.stop_signnum += 1
                self.start = rospy.Time.now()

                self.stopcounter.publish(self.stop_signnum)
                print('Stop line #',self.stop_signnum)
        
        key = cv2.waitKey(1) & 0xFF
        
        
        # if the `q` key was pressed, exit.
        if key == ord("q"):
            exit()


if __name__ == "__main__":
    try:
        nod = LaneDetection()
    except rospy.ROSInterruptException as e:
        print(e)