import os
import sys
import time
import smbus
import datetime

import serial
import re

    
from imusensor.MPU9250 import MPU9250
import argparse
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="IMU Record-{}.csv".format(datetime.datetime.now()),
	help="path to output CSV file containing IMU scan")
ap.add_argument("-c", "--calibration", type=bool, default=False,
	help="calibration, default=False")
args = vars(ap.parse_args())

imu_readings = np.zeros(14)
calibrate = args["calibration"]
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
ser = serial.Serial("/dev/ttyACM0", 19200)
if calibrate:
	imu.caliberateGyro()
	imu.caliberateAccelerometer()
	imu.caliberateMagPrecise()
	print ("Mag calibration Finished")
	print (imu.MagBias)
	print (imu.Magtransform)
	print (imu.Mags)
	imu.saveCalibDataToFile("/home/alphatrion/IMUdata/calib_real4.json")
else:
	imu.loadCalibDataFromFile("/home/alphatrion/IMUdata/calib_real4.json")
	lastRead = time.time()
	while True:
		
		if time.time() - lastRead >= 0.1:
			imu.readSensor()
			imu.computeOrientation()
			

			print(imu.AccelVals, imu.GyroVals, imu.MagVals)

			# print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))
			imu_readings[0] = np.array([time.time()])
			imu_readings[1:4] = np.array(imu.AccelVals)
			imu_readings[4:7] = np.array(imu.GyroVals)
			imu_readings[7:10] = np.array(imu.MagVals)
			imu_readings[10] = np.array(imu.roll)
			imu_readings[11] = np.array(imu.pitch)
			imu_readings[12] = np.array(imu.yaw)
			lastRead = time.time()

		bs = ser.readline()
		enc_read = re.findall("\d+\.\d+", str(bs))
		if len(enc_read) > 0:
			imu_readings[13] = np.array(float(enc_read[0]))
		# time.sleep(0.1)
		with open(args["output"], "ab") as ff:
			np.savetxt(ff, np.expand_dims(imu_readings, axis=0),fmt='%4.8f' , delimiter=",")
