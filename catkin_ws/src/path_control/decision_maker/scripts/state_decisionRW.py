#!/usr/bin/env python3
import rospy
import math
import serial
import re

from std_msgs.msg import String, Int16, Byte, Float32, Int32MultiArray

from utils.msg import localisation, IMU
import json

from custom_msg.msg import Omnidetection

from enum import IntEnum # for traffic lights

left_curv_dist = 162.577 # (long_curve_radius = 103.5)*(pi/2 -> 1/4 of circle)
right_curv_dist = 104.458 +20 # (short_curve_radius = 66.5)*(pi/2 -> 1/4 of circle)
encoder_factor = 1 # (1 cm = 7.9 rotations)
act_vehicle_speed = 35 # (If speed is given as 12 cm/s, actual speed is around 35 cm/s)

class DecisionMaker():
    def __init__(self):
        self.start_car_firsttime = True
        self.traffic3 = True
        self.stop_distance = 39
        self.min_speed = 0.14
        self.max_speed = 0.22
        self.moving = True
        self.stopped = True
        self.current_speed = 0.0
        self.ped_area = 0.0
        self.cross_area = 0.0
        self.ped_area_max = 17000
        self.stopcounter= 1
        self.heading = 0.0
        self.manual_turn_counter = 0
        self.desired_heading = math.pi/2
        self.go_straight = True
        self.traffic_light = 0
        self.encoder_reading = 0.0
        self.traffic_light = [0, 0, 0, 0]
        self.s1 = False
        self.s2 = False
        self.s3 = False
        self.s4 = False
        # self.
        rospy.init_node('decision_maker_node', anonymous=True)
        self.steer(0.0)
        self.move(0.0)

        # self.ser = serial.Serial('/dev/ttyACM0', 19200)

        # Traffic lights
        self.tl_start = self.Color.RED
        self.tl_master = self.Color.RED
        self.tl_slave = self.Color.RED
        self.tl_antimaster = self.Color.RED
        
    def start_car(self):
        # print('start')
        print('Traffic light is Red/Yellow')
        if self.s3:
            print('Traffic Light is green')
            self.move(self.max_speed)
            self.start_car_firsttime = False
   #  TrafficLightColor
    class Color(IntEnum):
        RED     = 0
        YELLOW  = 1
        GREEN   = 2     

    def stop_pedestrain(self):
        
        command = {"action": "1", "speed": 0.0}
        command = json.dumps(command)
        command_pub = rospy.Publisher('/automobile/command', String, queue_size=1)
        
        command_pub.publish(command)
        
        
    def get_area(self, box):
        
        w = box[2] - box[0]
        h = box[3] - box[1]

        return w*h

    def keep_straight(self):
        diff = self.heading - self.desired_heading
        diff = diff % math.pi
        #print(diff)
        if abs(diff) < 0.1 :
            self.steer(diff*100)
            # print(diff)
    def distance(self, distance):
        enc_dist = distance*encoder_factor
        self.move(0.0)
        init_encoder = self.encoder_reading
        self.move(self.max_speed)
        while ((self.encoder_reading - init_encoder) < enc_dist):
            self.steer(0.0)
        # self.move(0.0)
        # rospy.sleep(10)
        

    def take_turn(self, direction):
        
        steering_command = [-14.2, 22.2]
        self.move(0.0)
        init_enc = self.encoder_reading
        print(init_enc)
        self.steer(0.0)
        
        if direction == "Right":
            print("Taking Right turn")
            self.move(self.max_speed)

            while ((self.encoder_reading -init_enc)< right_curv_dist*encoder_factor):
                # print((self.encoder_reading -init_enc), right_curv_dist*encoder_factor)
                self.steer(steering_command[1])
            # print('Encoder after right turn: ', self.encoder_reading)

            
        elif direction=="Left":
            print('Taking Left turn')
            self.distance(10)
            self.move(self.max_speed)

            while ((self.encoder_reading - init_enc )< (left_curv_dist*encoder_factor)):
                self.steer(steering_command[0])
            
        print('Encoder value after turning: ', self.encoder_reading - init_enc)
        # self.move(0.0)
        self.steer(0.0)

        print('stopped steering')

    def decisions(self):
        
        if self.ped_area > self.ped_area_max:
            if self.moving:
                rospy.loginfo("Pedestrian is detected, stopping for pedestrain.")
            self.stop_pedestrain()
            self.moving = False
        if (self.no_ped) & (self.moving == False):
            rospy.loginfo("Moving.")
            self.move(self.max_speed)
            self.moving = True


    def detection_callback(self, data):
        labels = data.labels
        bboxs = data.bboxs
        if 3 in labels:
            self.no_ped = False
            i = labels.index(3)
            self.ped_area = self.get_area(bboxs[i*4:(i+1)*4])
        else:
            self.no_ped = True
        self.decisions()
    

    def manual_maneuver(self):
        if self.stopcounter == 1:
            self.move(self.max_speed)
            self.distance(self.stop_distance)
            self.take_turn("Right")
            
        elif self.stopcounter in [2, 4]:
            self.distance(self.stop_distance)
            rospy.loginfo("Stopping for Stop sign.")
            self.move(0.0)
            rospy.sleep(3)
            self.take_turn("Left")
        elif self.stopcounter == 3:
            self.distance(self.stop_distance)
            self.move(0.0)
        elif self.stopcounter == 5:
            self.move(self.max_speed)
            self.distance(self.stop_distance)
            self.move(0.0)
            rospy.sleep(3.0)
            self.take_turn("Right")
            self.move(0.0)

    def lane_count(self, data):
        self.stopcounter = data.data
        print('Stop Line count: ', self.stopcounter)
        self.manual_maneuver()
        

    def update_heading(self, data):
        self.heading = 0.5*(data.rotA + data.rotB)
        if self.go_straight:
            self.keep_straight()

    def update_imu(self, data):
        if(self.stopcounter == 7):

            if(data.pitch <= -0.13 and self.manual_turn_counter == 0):
                self.manual_turn_counter = 1
            if(data.pitch >=  0.13 and self.manual_turn_counter == 1):
                self.manual_turn_counter = 2
            if(data.pitch <= 0.001 and self.manual_turn_counter == 2):
                self.move(0.2)
                rospy.sleep(1.5)
                self.take_turn("Right")
                # self.move(0.2)
                # rospy.sleep(1)
                self.manual_turn_counter = 0

                ## Final lane manual maneuver
                rospy.sleep(4)
                rospy.loginfo("Parking sign is detected")
                rospy.sleep(6)
                rospy.loginfo("Parking sign is detected")
                rospy.sleep(8)
                rospy.loginfo("Crosswalk sign is detected, Vehicle slowing down")
                self.move(0.2)
                rospy.sleep(5)
                self.move(0.0)
                rospy.loginfo("Vehicle stopped")
        # print(self.manual_turn_counter)
        
        
    def update_traffic_light(self, data):
        if self.start_car_firsttime:
            self.start_car()
        

        self.traffic_light = data.data
        if self.traffic_light[2] == 2:
            self.s3 = True
        else:
            self.s3 = False
        if self.traffic_light[3] == 2:
            self.s4 = True
        else:
            self.s4 = False

        if self.stopcounter == 3:
            print('Traffic light is red/yellow.')
            if self.traffic3:
                if self.s4:
                    print('Traffic Light is green')
                    self.move(self.max_speed)
                    self.traffic3 = False
            # rospy.loginfo("Priority sign is detected, not stopping")
        # print(self.traffic_light)


    def update_encoder_readings(self, data):
        self.encoder_reading = data.data

    def reset_encoder(self):
        self.encoder_reading = 0

    def command_publisher(self, command):
        command = json.dumps(command)
        r = rospy.Rate(10)
        command_pub = rospy.Publisher('/automobile/command', String, queue_size=1)
        
        command_pub.publish(command)
        r.sleep()

    def move(self, speed : float):
        command = {"action": "1", "speed": -speed}
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
        # rospy.Subscriber('/automobile/localisation', localisation, self.update_heading)
        # rospy.Subscriber('/automobile/IMU', IMU, self.update_imu)
        rospy.Subscriber('/automobile/trafficlight', Int32MultiArray, self.update_traffic_light)
        rospy.Subscriber('/sensor/encoder', Float32, self.update_encoder_readings)


        
        # self.decisions()

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
