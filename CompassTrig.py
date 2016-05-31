"""
Robotritons troubleshooting version for magnetometer navigation.

Purpose: Use a magnetometer to calculate the vehicle's cardinal heading.
Requirements: An InvenSense MPU-9250. The python modules spidev, time, math, navio.util, and navio.mpu9250
Use: Instantiate an imu object, initialize it, then call the read_mag() method to update the list of magnetometer_data.
	The angle of north, with reference from the front of the vehicle, is then calculated.

Updates:
- May 30, 2016. At unknown conditions, the magnetometer will start outputting all zeros.

Resources:
https://docs.emlid.com/navio/Navio-dev/read-gps-data/
https://shahriar.svbtle.com/importing-star-in-python
"""

import spidev
import time
import math
import navio.util

from navio.mpu9250 import MPU9250

navio.util.check_apm()

imu = MPU9250()
print "Connection established: ", imu.testConnection()

imu.initialize()

time.sleep(1)

while True:
	imu.read_mag()
	
	#Prepare variables
	if (imu.magnetometer_data[0] == 0):
		x = (imu.magnetometer_data[1]/(0.001))
	else:
		x = (imu.magnetometer_data[1]/imu.magnetometer_data[0])
	radians = math.acos(x)
    theta = math.degrees(radians)
	
	#Convert coordinates from cartesian to polar
    if ((imu.magnetometer_data[0] < 0) and (imu.magnetometer_data[1] < 0)):
		angle = theta + 180
    elif ((imu.magnetometer_data[0] < 0) and (imu.magnetometer_data[1] > 0)):
		angle = theta + 180
    elif ((imu.magnetometer_data[0] > 0) and (imu.magnetometer_data[1] < 0)):
		angle = theta + 360
    else:
		angle = theta + 0
	# print "Accelerometer: ", imu.accelerometer_data
	# print "Gyroscope:     ", imu.gyroscope_data
	# print "Temperature:   ", imu.temperature
	print "Magnetometer:  ", imu.magnetometer_data
	print "Angle in degrees: ", angle
	print "Corrected angle in degrees: ", theta

	time.sleep(0.5)
