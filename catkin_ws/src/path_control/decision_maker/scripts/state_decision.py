#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
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
    def get_area(self, box):
        
        w = box[2] - box[0]
        h = box[3] - box[1]

        return w*h

    def right_curve_short(self):
        curve_time = rospy.Time.now() + rospy.Duration(5.22)
        while(rospy.Time.now() <= curve_time):
            self.steer(21.61) #21.35

    def decisions(self):
        if self.ped_area > self.ped_area_max:
            self.move(0.0)
            # self.moving = False
        elif self.ped_area < 10000:
            self.move(0.20)
        if self.stopcounter == 1:
            self.curve1()

    def detection_callback(self, data):
        labels = data.labels
        bboxs = data.bboxs
        
        if 3 in labels:
            i = labels.index(3)
            self.ped_area = self.get_area(bboxs[i*4:(i+1)*4])
            print('Ped ', self.ped_area)
        self.decisions()


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
        rospy.init_node('decision_maker_node', anonymous=True)

        rospy.Subscriber("/perception/omni_detection", Omnidetection, self.detection_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

# if current_state == 'move':
#     if area_pedastrian 

if __name__ == '__main__':
    try:
        node = DecisionMaker()
        node.listener()

        
    except rospy.ROSInterruptException:
        pass
