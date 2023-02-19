#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from custom_msg.msg import Lanes

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    x1 = int((y1-intercept)/slope)
    y2 = int(y1*0.3)
    x2 = int((y2-intercept)/slope)
    return np.array([x1, y1, x2, y2, slope])

def average_slope_intercept(lane_image, lines):
    fit = []
    if lines.shape is None:
        return np.array([])
    for line in lines:
        x1 ,y1, x2, y2 =line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1) # y = mx + b
        slope = parameters[0]
        intercept = parameters[1]
        if abs(slope)>0.1 and abs(slope)<10: # remove horizontal and vertical lines
            fit.append((slope, intercept))
    if fit == []:
        return np.array([])
    else:
        fit_average = np.average(fit, axis=0)
        line = make_coordinates(lane_image, fit_average)
        return np.array([line])


def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def display_line(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            if len(line) >= 1:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 10)
    return line_image

def region_of_interest(image, half):
    height = image.shape[0]
    width = image.shape[1]
    crop_height = int(height*0.5)
    crop_width = int(width*0)
    half_width_offset = int(width*0.1)
    if half == 'left':
        polygons = np.array([
            [(crop_width, crop_height), (int(width/2)-half_width_offset, crop_height), (int(width/2)-half_width_offset, height), (crop_width, height)]
            ])
    elif half == 'right':
        crop_width_right = width- crop_width
        polygons = np.array([
            [(int(width/2)+half_width_offset, crop_height), (crop_width_right, crop_height), (crop_width_right, height), (int(width/2)+half_width_offset, height)]
            ])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

class LaneDetection():
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('LanesDetectionNode', anonymous=False)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.LanePub = rospy.Publisher('/lanes/lanes_detected', Lanes, queue_size=1)
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        canny_image_left = canny(self.cv_image)
        canny_image_right = np.copy(canny_image_left)
        cropped_image_left = region_of_interest(canny_image_left, 'left')
        cropped_image_right = region_of_interest(canny_image_right, 'right')
        # bin size: 2 pixels and one degrees; threshold: minimum intersection
        MinLineLength = 20
        MaxLineGap = 20
        bin_size_p = 2
        bin_size_theta = 2*(np.pi/180)
        threshold_num_points = 60
        lines_left = cv2.HoughLinesP(cropped_image_left, bin_size_p, bin_size_theta, threshold_num_points, np.array([]), minLineLength=MinLineLength, maxLineGap=MaxLineGap) 
        lines_right = cv2.HoughLinesP(cropped_image_right, bin_size_p, bin_size_theta, threshold_num_points, np.array([]), minLineLength=MinLineLength, maxLineGap=MaxLineGap)
        if lines_left is not None and lines_right is not None:
            average_lines_left = average_slope_intercept(self.cv_image, lines_left) # np.array([x1, y1, x2, y2, slope])
            average_lines_right = average_slope_intercept(self.cv_image, lines_right) # np.array([x1, y1, x2, y2, slope])
            msg_to_send = Lanes()
            try:
                squzzed_average_lines_left = np.copy(average_lines_left).squeeze()
                x2_left, y2_left, slope_left = squzzed_average_lines_left[2:]
                msg_to_send.left_lane =  (x2_left, y2_left, slope_left)# tuple ([x2, y2, slope]) -> tuple (x2, y2, slope)
            except ValueError:
                if average_lines_left.shape[0]==0:
                    msg_to_send.left_lane = (0, 0, 0)
            try:
                squzzed_average_lines_right = np.copy(average_lines_right).squeeze()
                x2_right, y2_right, slope_right = squzzed_average_lines_right[2:]
                msg_to_send.right_lane = (x2_right, y2_right, slope_right)
            except ValueError:
                if average_lines_right.shape[0]==0:
                    msg_to_send.right_lane = (0, 0, 0)
            self.LanePub.publish(msg_to_send)
            
            # Visualization
            if average_lines_left.shape[0]==0:
                concat_lines = average_lines_right
                lines_image = display_line(self.cv_image, concat_lines[:-1])
            elif average_lines_right.shape[0]==0:
                concat_lines = average_lines_left
                lines_image = display_line(self.cv_image, concat_lines[:-1])
            else:
                concat_lines = np.vstack((average_lines_left, average_lines_right))
                lines_image = display_line(self.cv_image, concat_lines[:, :-1])
            combo_image = cv2.addWeighted(self.cv_image, 0.6, lines_image, 1, 1)
            cv2.imshow('result', combo_image)
            cv2.waitKey(1)
        else:
            print("\033[1;33m Warning: \033[0m no lanes detected!")
            cv2.imshow('result', self.cv_image)
            cv2.waitKey(1)

            
if __name__ == '__main__':
    try:
        nod = LaneDetection()
    except rospy.ROSInterruptException:
        pass