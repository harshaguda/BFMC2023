#!/usr/bin/env python3
import math
import rospy

from generate_command import generate_command
from command_publisher import command_publisher
from state_publisher import state_publisher
from steering_model import AckermannModel

VEHICLE_IGNITION		    = False
INITIAL_VEHICLE_SPEED   	= 0.0
START_VEHICLE_SPEED 		= 12.0
STEERING_OFFSET    		    = -2.0

def curvature_callback(data):
    curvature_value = data
    ## Yet to be updated

def decision_maker():
    rospy.init_node('decision_maker', anonymous=True)
    
    ### Vehicle initial state
    current_speed = INITIAL_VEHICLE_SPEED
    initial_command = generate_command(1, current_speed)
    command_publisher(initial_command)
    
    ### Vehicle start
    while True:
    	value = input("Say the word: ")
    	if value == "at":
        	print("GO ALPHA TRION!!!")
        	VEHICLE_IGNITION = True
        	break
    	else:
        	print("Wrong key word, try again!")
    
    if VEHICLE_IGNITION:
        current_speed = START_VEHICLE_SPEED
    	command = generate_command(1, current_speed)
    	command_publisher(command)
    	
    vehicle_model = AckermannModel(STEERING_OFFSET)
    
    while not rospy.is_shutdown():
        curvature = rospy.Subscriber('*** CURVATURE PUB MSG ***', Int, curvature_callback)
        steering_value = vehicle_model.calculate_steering_command(curvature)
        steering_command = generate_command(2, steering_value)
        command_publisher(steering_command)
        state_publisher(current_speed, steering_command, curvature)
        rate.sleep()

if __name__ == '__main__':
    try:
        decision_maker()
    except rospy.ROSInterruptException:
        pass
