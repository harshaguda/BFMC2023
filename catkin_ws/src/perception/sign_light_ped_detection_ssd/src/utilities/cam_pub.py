#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


class CameraPub():
    def __init__(self):
        rospy.init_node('Campubnode', anonymous=False)     
        self.publisher = rospy.Publisher("/automobile/image_raw", Image, queue_size=1)
        self.video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)        

    def run(self):
        while True:
            try:
                ret_val, image = self.video_capture.read()
                print(image.shape)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                imageObject = CvBridge().cv2_to_imgmsg(image, "bgr8")
                imageObject.header.stamp = rospy.Time.now()
                self.publisher.publish(imageObject)
            #cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == 27:
                    break
            except:
                break
if __name__ == '__main__':
    try:
        nod = CameraPub()
        nod.run()
    except rospy.ROSInterruptException:
        exit()
