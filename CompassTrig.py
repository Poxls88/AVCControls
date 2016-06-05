"""
Robotritons troubleshooting version for magnetometer navigation.

Purpose: Use a magnetometer to calculate the vehicle's cardinal heading.
Requirements: An InvenSense MPU-9250. The python modules spidev, time, math, navio.util, and navio.mpu9250
Use: Place the vehicle facing north. Instantiate an imu object, initialize it, then call the read_mag() method to update the list of magnetometer_data.
	The angle of north, with reference from the front of the vehicle, is then calculated.

Updates:
- June 5, 2016. Created "navio/mpu9250_better" and probably stopped magnetometer from returning all zeros after a couple of program restarts. I made this band-aid
	solution following the progress of the Emlid Developers linked to under "Resources Bugs". The root of the problem is an I2C interruption leading to a
	device unknown state. So far I have restarted the CompassTrig.py several times while it includes mpu9250_better and the magnetometer hasn't produced all zeros. 
- June 2, 2016. Header is now calculated after a calibration method and then using normalized vectors on a unit circle. This prevents trig domain errors.
- May 30, 2016. At unknown conditions, the magnetometer will start outputting all zeros.

Resources:
https://docs.emlid.com/navio/Navio-dev/mpu9250-imu/
https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf
https://shahriar.svbtle.com/importing-star-in-python
http://stackoverflow.com/questions/1621831/how-can-i-convert-coordinates-on-a-square-to-coordinates-on-a-circle

Resources Bugs:
https://community.emlid.com/t/bug-magnetometer-reading-process-fail/688 #Community is aware of problem, but only C++ fixes are implemented
https://github.com/ArduPilot/ardupilot/pull/2487 #The immediate solution was to stop the AK8963 device reset. This is the solution I mimicked with "mpu9250_better"
https://github.com/ArduPilot/ardupilot/pull/2493 #Emlid developers have fixed the C++ driver
https://github.com/ArduPilot/ardupilot/pull/2504 #Someone else did it their own way for C++
"""

import spidev
import time
import math
import navio.util

from navio.mpu9250_better import MPU9250

navio.util.check_apm()
imu = MPU9250()
print "Connection established: ", imu.testConnection()

#initialize communication
imu.initialize()
time.sleep(1)


def calibrateUnit():
	xSet = []
	ySet = []
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		print xSet #
		time.sleep(0.5)
	xMean = float(sum(xSet))/max(len(xSet),1)
	print "now Y"
	time.sleep(5)
	for x in range(10):
		imu.read_mag()
		ySet.append(imu.magnetometer_data[1])
		print ySet #
		time.sleep(0.5)
	yMean = float(sum(ySet))/max(len(ySet),1)
	Unit = [xMean,yMean]
	print "Unit: ", Unit #
	return Unit

#Prepare variables
#Any trig needs to be performed on a unit circle
unit = calibrateUnit()

while True:
	#Read our magnetometer
	#	Note: The magnetometer data is stored as a list ordered [x,y,z]
	#	Note: x+ is directed towards the front of the RPI2/Navio+ and y+ is directed towards the right of the RPI2/Navio+
	#	Note: all calculations assume x is the verticle axis and y is horizontal
	imu.read_mag()
	
	#Scale readings into a unit square relative to their maximum...
	xVal = min((imu.magnetometer_data[0]/unit[0]),unit[0])
	yVal = min((imu.magnetometer_data[1]/unit[1]),unit[1])
	#Map the square to a unit circle
	print "X raw: %f \n scaled: %f" % (imu.magnetometer_data[0],xVal)
	print "Y raw: %f \n scaled: %f" % (imu.magnetometer_data[1],yVal)
	'''
	xCircle = float(xVal * math.sqrt(1 - 0.5*(math.pow(yVal,2))))
	yCircle = float(yVal * math.sqrt(1 - 0.5*(math.pow(xVal,2))))
	#Normalize the coordinates so that they are essentially on the edge of a unit circle
	mag = float(math.sqrt((math.pow(xCircle,2)) + (math.pow(yCircle,2))))
	xNorm = xCircle/mag
	yNorm = yCircle/mag
	
	#Convert placement on the unit circle to an angle
	radians = math.acos(xNorm)#Although y is our horizontal, we compute arccosine of x to get an angle relative to forwards
	theta = math.degrees(radians)
	
	#Convert arccos domain to full circle
	if (yCircle > 0): #If reading is right of forwards
		angle = theta + 180 #Then our arccos is really on the other side of the circle
	else:
		angle = theta + 0 #Then our arccos is on the left of the circle and 0-180 relative to forwards
	# print "Accelerometer: ", imu.accelerometer_data
	# print "Gyroscope:     ", imu.gyroscope_data
	# print "Temperature:   ", imu.temperature
	print "Magnetometer:  ", imu.magnetometer_data
	print "Arccos in degrees: ", theta
	print "Corrected angle in degrees: ", angle
	'''
	time.sleep(0.5)
