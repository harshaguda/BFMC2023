#!/usr/bin/env python3
from cpu_vision.ssd.mobilenetv1_ssd import create_mobilenetv1_ssd, create_mobilenetv1_ssd_predictor
import cv2
import numpy as np
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msg.msg import Omnidetection
import torch
class LaneDetection():
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        # device=torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # device=torch.device("cuda:0")
        device=torch.device("cpu")
        pkg_path = os.path.dirname(os.path.abspath(__file__))
        model_path =os.path.join(pkg_path, 'models/RealandSimul/mb1-ssd-Epoch-308-Loss-1.0236728725892104.pth')
        # model_path =os.path.join(pkg_path, 'models/RealandSimul/mb1-ssd-Epoch-1709-Loss-0.8591486492058199.pth') 
        label_path = os.path.join(pkg_path, 'models/RealandSimul/labels.txt')
        self.class_names = [name.strip() for name in open(label_path).readlines()]
        net = create_mobilenetv1_ssd(len(self.class_names), is_test=True, device=device)
        net.load(model_path)
        self.predictor = create_mobilenetv1_ssd_predictor(net, candidate_size=200, device=device)
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('OmniDetectionNode', anonymous=False)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.DetectionPub = rospy.Publisher('/perception/omni_detection', Omnidetection, queue_size=1)
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        bboxs, labels, probs = self.predictor.predict(image, 10, 0.4)
        # Pub
        msg_to_send = Omnidetection()
        a = bboxs.numpy().astype(float)
        msg_to_send.bboxs = bboxs.numpy().astype(float).flatten()
        msg_to_send.labels = labels.numpy().astype(int)
        msg_to_send.probs = probs.numpy().astype(float)
        self.DetectionPub.publish(msg_to_send)
        # rospy.Rate(10).sleep()
        # Visualization
        for i in range(bboxs.size(0)):
            box = bboxs[i, :].numpy().astype(int)
            cv2.rectangle(self.cv_image, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 4)
            label = f"{self.class_names[labels[i]]}: {probs[i]:.2f}"
            cv2.putText(self.cv_image, label,
                        (box[0] + 20, box[1] + 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,  # font scale
                        (255, 0, 255),
                        2)  # line type
        cv2.imshow('result', self.cv_image)
        cv2.waitKey(1)
        #print(f"Found {len(probs)} objects.")


if __name__ == '__main__':
    try:
        nod = LaneDetection()
    except rospy.ROSInterruptException:
        pass
