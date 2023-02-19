#!/usr/bin/env python3
 
import argparse
import os
import random
import time
import shutil
import warnings
import datetime
import torch
import torchvision
import torch.nn as nn
import torch.nn.parallel
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
import torch.optim
import torch.utils.data
import torchvision.datasets as datasets
import torchvision.transforms as transforms
import torchvision.models as models
from reshape import reshape_model
import cv2
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
from std_msgs.msg import String
import rospy

class Inference():
    # ===================================== INIT==========================================
    def __init__(self):
        # sub
        self.folder_path = "tmp"
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        arch = 'resnet18'
        resolution = 224
        self.model = models.__dict__[arch]()
        self.model = reshape_model(self.model, arch, 3)
        model_path = torch.load("models/track/model_best.pth.tar")
        self.model.load_state_dict(model_path['state_dict'])
        # setup data transformations
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                            std=[0.229, 0.224, 0.225])
        self.train_transforms = transforms.Compose([
            transforms.RandomResizedCrop(resolution),
            transforms.RandomHorizontalFlip(),
            transforms.ToTensor(),
            normalize,
        ])
        # ROS
        rospy.init_node('Inferencenode', anonymous=False)   
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)  
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
        # pub
        # initially move forward
        self.steerAngle = 20
        self.speed = 0.15
        command = {}
        command['action']='1'
        command['speed']=float(self.speed)
        command = json.dumps(command)
        self.publisher.publish(command)
        rospy.spin()

    def inference(self):
        train_dataset = datasets.ImageFolder(self.folder_path, self.train_transforms)
        train_loader = torch.utils.data.DataLoader(
                train_dataset, batch_size=1, shuffle=True,
                num_workers=1, pin_memory=True)
        dataiter = iter(train_loader)
        images, _ = next(dataiter)
        outputs = self.model(images)
        _, predicted = torch.max(outputs, 1)
        print("outputs", outputs)
        print("predicted", predicted)
        return predicted

    def track(self):
        predicted = self.inference()                                                                  
        command = {}
        # go straight #i/k
        if predicted == 2:
            command['action']        =  '2'
            command['steerAngle']    =  -float(0.0)
        # turn left #j
        if predicted == 0:
            command['action']        =  '2'
            command['steerAngle']    =  -float(self.steerAngle)
        # turn right #l
        if predicted == 1:
            command['action']        =  '2'
            command['steerAngle']    =  float(self.steerAngle)
        command = json.dumps(command)
        self.publisher.publish(command)   

    def image_callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img_dir = os.path.join(self.folder_path, "ik/currentFrame.jpg")
        cv2.imwrite(img_dir, self.cv_image)
        img_dir = os.path.join(self.folder_path, "j/currentFrame.jpg")
        cv2.imwrite(img_dir, self.cv_image)
        img_dir = os.path.join(self.folder_path, "l/currentFrame.jpg")
        cv2.imwrite(img_dir, self.cv_image)
        print("ggg")
        self.track()
        
if __name__ == '__main__':
    try:
        nod = Inference()
    except rospy.ROSInterruptException:
        pass