#!/usr/bin/env python3
import rospy
import math

from std_msgs.msg import String, Int16
from utils.msg import localisation
import json

from custom_msg.msg import Omnidetection


class DecisionMaker():
    def __init__(self):
        self.moving = False
        self.stopped = True
        self.current_speed = 0.0
        self.ped_area = 0.0
        self.cross_area = 0.0
        self.ped_area_max = 17000
        self.stopcounter= 0
        self.heading = 0.0
        rospy.init_node('decision_maker_node', anonymous=True)
        self.steer(0.0)
        self.move(0.20)
        
    def get_area(self, box):
        
        w = box[2] - box[0]
        h = box[3] - box[1]

        return w*h

    def right_curve_short(self):
        curve_time = rospy.Time.now() + rospy.Duration(10.72)
        rospy.sleep(1.5)
        self.move(0.12)
        print('start steering')
        #while(rospy.Time.now() <= curve_time):
        while not (self.heading <= 0.1):
            self.steer(21.61) #21.35
        while not (self.heading <= 0.01):
            self.steer(15.61) #21.35
        self.move(0.2)
        self.steer(0.0)
        print('stopped steering')

    def take_turn(self, direction):
        rospy.sleep(1.6) # Time to aligh rear axle to stop line
        self.move(0.1)
        steering_command = [21.61, -17.14]
        print('start steering')
        current_heading_pose = round(self.heading/(math.pi/2))
        if direction=="Right": 
            desired_heading = (current_heading_pose-1)*(math.pi/2)
            while not (-1*(desired_heading+0.1) <= self.heading <= desired_heading+0.1):
                self.steer(steering_command[0])
            while not (-1*(desired_heading+0.01) <= self.heading <= desired_heading+0.01):
                self.steer(0.75*steering_command[0])
        elif direction=="Left":
            desired_heading = (current_heading_pose+1)*(math.pi/2)
            while not (-1*(desired_heading+0.1) <= self.heading <= desired_heading+0.1):
                self.steer(steering_command[1])
            while not (-1*(desired_heading+0.01) <= self.heading <= desired_heading+0.01):
                self.steer(0.75*steering_command[1])
        print('stopped steering')

    def decisions(self):
        if self.ped_area > self.ped_area_max:
            self.move(0.0)
            # self.moving = False
        elif self.ped_area < 10000:
            self.move(0.20)
        if self.stopcounter == 1:
            print('steer')
            #self.right_curve_short()
            self.take_turn("Right")
        if self.stopcounter == 2:
            print('steer')
            #self.right_curve_short()
            self.take_turn("Right")

    def detection_callback(self, data):
        labels = data.labels
        bboxs = data.bboxs
        
        if 3 in labels:
            i = labels.index(3)
            self.ped_area = self.get_area(bboxs[i*4:(i+1)*4])
            print('Ped ', self.ped_area)
        
    def lane_count(self, data):
        self.stopcounter = data.data
        if self.stopcounter == 1:
            #self.right_curve_short()
            self.take_turn("Right")
        elif self.stopcounter == 2:
            #self.right_curve_short()
            self.take_turn("Left")
        # print(self.stopcounter)

    def update_heading(self, data):
        self.heading = 0.5*(data.rotA + data.rotB)

    def command_publisher(self, command):
        command = json.dumps(command)
        
        command_pub = rospy.Publisher('/automobile/command', String, queue_size=10)
        
        command_pub.publish(command)

    def move(self, speed : float):
        command = {"action": "1", "speed": speed}
        self.command_publisher(command)

    def steer(self, steer : float):
        command = {"action": "2", "steerAngle": steer}
        self.command_publisher(command)

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        
        rospy.Subscriber("/perception/omni_detection", Omnidetection, self.detection_callback)
        rospy.Subscriber('/lanes/stop_line', Int16, self.lane_count)
        rospy.Subscriber('/automobile/localisation', localisation, self.update_heading)
        # spin() simply keeps python from exiting until this node is stopped

        # self.decisions()

        rospy.spin()

# if current_state == 'move':
#     if area_pedastrian 

if __name__ == '__main__':
    try:
        node = DecisionMaker()
        node.listener()

        
    except rospy.ROSInterruptException:
        pass
