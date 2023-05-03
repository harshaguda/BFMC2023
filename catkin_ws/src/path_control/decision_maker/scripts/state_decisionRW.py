#!/usr/bin/env python3
import rospy
import math

from std_msgs.msg import String, Int16, Byte, Float32
from utils.msg import localisation, IMU
import json

from custom_msg.msg import Omnidetection

left_curv_dist = 162.577 # (long_curve_radius = 103.5)*(pi/2 -> 1/4 of circle)
right_curv_dist = 104.458 # (short_curve_radius = 66.5)*(pi/2 -> 1/4 of circle)
encoder_factor = 8.1 # (1 cm = 7.9 rotations)
act_vehicle_speed = 35 # (If speed is given as 12 cm/s, actual speed is around 35 cm/s)

class DecisionMaker():
    def __init__(self):
        self.min_speed = 0.14
        self.max_speed = 0.14
        self.moving = True
        self.stopped = True
        self.current_speed = 0.0
        self.ped_area = 0.0
        self.cross_area = 0.0
        self.ped_area_max = 17000
        self.stopcounter= 0
        self.heading = 0.0
        self.manual_turn_counter = 0
        self.desired_heading = math.pi/2
        self.go_straight = True
        self.traffic_light = 0
        self.encoder_reading = 0.0
        # self.
        rospy.init_node('decision_maker_node', anonymous=True)
        self.steer(0.0)
        self.move(0.14)
    
   
        

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
        
    def take_turn(self, direction):
        self.go_straight = False
        #self.move(0.14)
        # rospy.sleep(1.2) # Time to aligh rear axle to stop line
        #self.move(self.min_speed)
        steering_command = [21.61, -14.2]
        #print('start steering')
        #if direction=="Right": 
        #    self.desired_heading = ((current_heading_pose-1)*(math.pi/2)) % math.pi
           
            #print(self.desired_heading)
         #   while not (self.desired_heading-0.1 <= self.heading <= self.desired_heading+0.1):
          #      self.steer(steering_command[0])
           # while not (self.desired_heading-0.015 <= self.heading <= self.desired_heading+0.015):
            #    self.steer(0.3*steering_command[0])
        if direction == "Right":
            print("Taking Right turn")
            self.move(0.14)
            rospy.sleep(0.7)
            self.move(0.14)
            print("Encoder reading so far:", self.encoder_reading)
            self.encoder_reading = 0
            self.reset_encoder()
            while (self.encoder_reading < right_curv_dist*encoder_factor*20):
                self.steer(steering_command[1])
            print(self.encoder_reading)
        elif direction=="Left":
            print('Taking Left turn')
            self.move(0.14)
            rospy.sleep(0.7) 
            self.move(0.14)
            print('Encoder reading so far:', self.encoder_reading)
            self.encoder_reading = 0
            self.reset_encoder()
            print('Encoder reset done: ', self.encoder_reading)
            #self.move(0.2)
            while (self.encoder_reading < (left_curv_dist*encoder_factor*20)):
                self.steer(steering_command[1])
            #rospy.sleep(1.2)
            #self.steer(0.3*steering_command[1])
            #rospy.sleep(1.5)
        # self.move(self.max_speed)
        print('Encoder value after turning: ', self.encoder_reading)
        self.move(0.0)
        self.steer(0.0)
        self.steer(0.0)

        #self.steer(0.0)

        self.go_straight = True
        print('stopped steering')

    def decisions(self):
        # print(self.moving, self.no_ped)
        if self.ped_area > self.ped_area_max:
            if self.moving:
                rospy.loginfo("Pedestrian is detected, stopping for pedestrain.")
            self.stop_pedestrain()
            self.moving = False
            
            # self.moving = True
        if (self.no_ped) & (self.moving == False):
            rospy.loginfo("Moving.")
            self.move(self.max_speed)
            self.moving = True


    def detection_callback(self, data):
        labels = data.labels
        bboxs = data.bboxs
        # print(labels)
        if 3 in labels:
            self.no_ped = False
            i = labels.index(3)
            self.ped_area = self.get_area(bboxs[i*4:(i+1)*4])
            #print('Ped ', self.ped_area)
        else:
            self.no_ped = True
        self.decisions()
        
    def manual_maneuver(self):
        if self.stopcounter == 1:
            self.move(self.max_speed)
        elif self.stopcounter in [2, 3, 7]:
            rospy.loginfo("Stopping for Stop sign.")
            self.move(0.0)
            rospy.sleep(2.6)
            self.move(self.max_speed)
        elif self.stopcounter == 4:
            rospy.loginfo("Priority sign is detected, not stopping")

    def lane_count(self, data):
        self.stopcounter = data.data
        self.manual_maneuver()
        if self.stopcounter == 1:
            self.take_turn("Left")

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
        self.traffic_light = data.data     

    def update_encoder_readings(self, data):
        self.encoder_reading += data.data

    def reset_encoder(self):
        self.encoder_reading = 0

    def command_publisher(self, command):
        command = json.dumps(command)
        r = rospy.Rate(10)
        command_pub = rospy.Publisher('/automobile/command', String, queue_size=1)
        
        command_pub.publish(command)
        r.sleep()

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
        rospy.Subscriber('/automobile/IMU', IMU, self.update_imu)
        rospy.Subscriber('/automobile/trafficlight/master', Byte, self.update_traffic_light)
        rospy.Subscriber('/sensor/encoder', Float32, self.update_encoder_readings)
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
