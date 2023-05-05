#!/usr/bin/env python3

import time
import smbus

import rospy

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
    
from imusensor.MPU9250 import MPU9250
import numpy as np

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

#imu.loadCalibDataFromFile("/home/alphatrion/IMUdata/calib_real4.json")

class IMUNode():
	def __init__(self):
		self.prev_yaw = 0
		self.pub = rospy.Publisher('/localization/IMU', Imu, queue_size=1)
		rospy.init_node('IMUNode', anonymous=False)

 

	def getIMUData(self):
		while not rospy.is_shutdown():
			imu.readSensor()
			imu.computeOrientation()
			imu_data = Imu()
			imu_data.header.stamp = rospy.get_rostime()
			imu_data.header.frame_id = "MPU-9250"
			q = quaternion_from_euler(imu.roll, imu.pitch, imu.yaw)
			

			roll = np.arctan((2*(q[0]*q[1] + q[2]*q[3]))/(1 - 2*(q[1]**2 + q[2]**2)))
			pitch = np.arcsin(2*(q[0]*q[2] - q[3]*q[1]))
			yaw = np.arctan((2*(q[0]*q[3] + q[1]*q[2]))/(1 - 2*(q[2]**2 + q[3]**2)))
			print(roll, pitch, self.prev_yaw*0.85 + 0.15*yaw)
			imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w = q[0], q[1], q[2], q[3]
		
			imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z = imu.AccelVals

			imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z = imu.GyroVals
			
			self.prev_yaw = yaw
			self.pub.publish(imu_data)
			time.sleep(0.1)

if __name__ == "__main__":
    imuNode = IMUNode()
    imuNode.getIMUData()

	
	
	
