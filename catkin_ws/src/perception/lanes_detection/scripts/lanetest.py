#!/usr/bin/env python3

# import rospy
import cv2
import numpy as np

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2


# import matplotlib.pyplot as plt

def sobel_x(img):
    scale = 1
    delta = 0
    ddepth = cv2.CV_16S
    s_thresh=(100, 255)
    sx_thresh=(15, 255)
    

    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(float)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    h_channel = hls[:,:,0]

    grad_x = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1)
    abs_sobelx = np.absolute(grad_x) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    sxbinary = np.zeros_like(scaled_sobel)

    sxbinary[(scaled_sobel >= 15) & (scaled_sobel <= 255)] = 1

    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1

    color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255

    combined_binary = np.zeros_like(sxbinary)
    combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
    
    return combined_binary

def dilate(img, kernel=np.ones((5, 5), np.uint8)):
 
    img_dilation = cv2.dilate(img, kernel, iterations=1)
    return img_dilation
def erode(img, kernel=np.ones((5, 5), np.uint8)):
    img_erode = cv2.erode(img, kernel, iterations=1)
    return img_erode

def get_lanes(frame, stats, labels, idxs):
    crp_img = np.zeros(labels.shape)
    crp_img = (crp_img != 0)
    xs = []
    for i in idxs:
        crp_img = crp_img | (labels == i)
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        
        xs.append(x)
        xs.append(x+w)
    if crp_img.sum() < 500:
        return frame, []
    if sum(crp_img.nonzero()[1]) == 0:
        return frame, []
    
    
    min_x = min(xs)
    max_x = max(xs)

    plotx = np.linspace(min_x, max_x)
    coeff = np.polyfit(crp_img.nonzero()[1],crp_img.nonzero()[0], 2)
    p = np.poly1d(coeff)

    draw_x = np.linspace(min_x, max_x+15)#np.linspace(0,320)
    draw_y = p(draw_x) + 100

    draw_points = (np.asarray([draw_x, draw_y]).T).astype(np.int32)
    cv2.polylines(frame, [draw_points], False, (0,255,0), thickness=4 )

    return frame, draw_points

def vid_pipeline(frame):
    
    cropped_image = frame[40:180,:]
    cbe = sobel_x(cropped_image)
    # cbd = dilate(cb)
    # cbe = erode(cbd)
    output = cv2.connectedComponentsWithStats(cbe, 40, cv2.CV_32S)
    (numLabels, labels, stats, centroids) = output
    
    cls = Cluster().get_clusters(centroids, stats, 45)
    
    for val in cls:
        cropped_image, dp= get_lanes(cropped_image,stats, labels, cls[val])
        
    if cropped_image.shape[0] == 0:
        print('zero')
    return cropped_image

class Cluster():
    # Debug this class
    def __init__(self):
        
        self.clusterid = 0
        self.clusterid_max = 0
        self.clusters = dict()
        self.first = 1
        self.excempt_clusters = {0}
    def get_clusters(self, centroids, stats, thres=80):
        for i in range(1,centroids.shape[0]):
            dist = np.sqrt(((centroids - centroids[i])**2).sum(axis=1))
            # print(dist)
            idx = np.where((dist < thres))[0]
            idx = set(idx.tolist())
            area = stats[i, cv2.CC_STAT_AREA]
            # print(idx)
            if area < 50:
                # print(i, area)
                self.excempt_clusters = self.excempt_clusters | {i}
                # print(self.excempt_clusters)
                continue
            # clusterid = self.clusterid
            
            if self.first:
                # print(idx, self.clusterid)
                self.clusters[self.clusterid] = idx# - self.excempt_clusters
                self.first = 0
            else:
                self.clusterid, flag = self.get_clusterid(idx)
                # print(idx, self.clusterid, flag)
                
                if flag:
                    self.clusters[self.clusterid] = idx #- self.excempt_clusters
                else:
                    self.clusters[self.clusterid] = self.clusters[self.clusterid] | idx #- self.excempt_clusters
            if self.clusterid > self.clusterid_max:
                self.clusterid_max = self.clusterid
            # self.clusterid = clusterid
        for val in self.clusters:
            self.clusters[val] = self.clusters[val] - self.excempt_clusters
        return self.clusters
        

    def get_clusterid(self,idx):
        for val in self.clusters:
            if len(idx & self.clusters[val]) > 0:
                
                self.clusterid = val
                flag = 0
                return self.clusterid, flag
        
        self.clusterid_max += 1
        
        flag = 1
        return self.clusterid_max, flag



            
if __name__ == '__main__':
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

        cropped_image = image[100:180,:]
        cbe = sobel_x(cropped_image)
        # cbd = dilate(cb)
        # cbe = erode(cbd)
        output = cv2.connectedComponentsWithStats(cbe, 40, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = output
        
        cls = Cluster().get_clusters(centroids, stats, 45)
        
        for val in cls:
            cropped_image, dp= get_lanes(cropped_image,stats, labels, cls[val])
            
        if cropped_image.shape[0] == 0:
            print('zero')
        # combo_image = cv2.addWeighted(image, 0.6, cropped_image, 1, 1)
        cv2.imshow('result', cropped_image)
        # cv2.waitKey(1)
        # show the frame
        # cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break