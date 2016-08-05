"""
Robotritons testing version of compass navigation.

Purpose: Use a magnetometer to reliably steer the vehicle.
Requirements: An InvenSense MPU-9250. The python modules spidev, time, math, navio.util, and navio.mpu9250
Use: Place the vehicle facing north. Instantiate an imu object, then initialize it, then calibrate N,E,S,W, finally call the read_mag() method to update the list of magnetometer_data
	and calculate the current direction.
	Input a desired direction and the vehicle will try to turn itself that way.

Updates:
- August 5, 2016 Created the file.

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

import sys
import spidev
import time
import math
import navio.util

from navio.mpu9250_better import MPU9250
import VehiclePWMModule

navio.util.check_apm()
imu = MPU9250()
print "Connection established: ", imu.testConnection()

#initialize communication
imu.initialize()
time.sleep(1)
#initialize the servo and esc
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
#initialize desired direction
desDir = 0 #0=N, 1=E, 2=S, 3=W
curDir = 0

def calibrateCardinalMean():
	xSet = []
	ySet = []
	print "Place North"
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		#print xSet
		time.sleep(0.5)
	xNorth = float(sum(xSet))/max(len(xSet),1)
	yNorth = float(sum(ySet))/max(len(ySet),1)
	time.sleep(2)

        print "Place East"
        for x in range(10):
                imu.read_mag()
                xSet.append(imu.magnetometer_data[0])
                ySet.append(imu.magnetometer_data[1])
                #print xSet
                time.sleep(0.5)
        xEast = float(sum(xSet))/max(len(xSet),1)
        yEast = float(sum(ySet))/max(len(ySet),1)
	time.sleep(2)

        print "Place South"
        for x in range(10):
                imu.read_mag()
                xSet.append(imu.magnetometer_data[0])
                ySet.append(imu.magnetometer_data[1])
                #print xSet
                time.sleep(0.5)
        xSouth = float(sum(xSet))/max(len(xSet),1)
        ySouth = float(sum(ySet))/max(len(ySet),1)
	time.sleep(2)

        print "Place West"
        for x in range(10):
                imu.read_mag()
                xSet.append(imu.magnetometer_data[0])
                ySet.append(imu.magnetometer_data[1])
                #print xSet
                time.sleep(0.5)
        xWest = float(sum(xSet))/max(len(xSet),1)
        yWest = float(sum(ySet))/max(len(ySet),1)
	cardinalMean = {'xN':xNorth, 'yN':yNorth, 'xE':xEast, 'yE':yEast, 'xS':xSouth, 'yS':ySouth, 'xW':xWest, 'yW':yWest}
	time.sleep(0.5)
	return cardinalMean

#Calculate our average direction values
cardinalMean = calibrateCardinalMean()


while True:
	try:
		vehicle_esc.stop()
		vehicle_esc.rest()
		vehicle_servo.center()

		#Read our magnetometer
		#	Note: The magnetometer data is stored as a list ordered [x,y,z]
		#	Note: x+ is directed towards the front of the RPI2/Navio+ and y+ is directed towards the right of the RPI2/Navio+
		#	Note: all calculations assume x is the verticle axis and y is horizontal. Upsidedown vehicle reverses E<->W
		imu.read_mag()
	
		#f = open('CompassCapt.txt', 'w')
		xRaw = imu.magnetometer_data[0] #print >> f, "X raw, %f" % (imu.magnetometer_data[0])
		yRaw = imu.magnetometer_data[1] #print >> f, "Y raw, %f" % (imu.magnetometer_data[1])
		#f.close()
	
		if (abs(xRaw-yRaw) < 15): #If NORTH or EAST (both have similar x/y values)
			if (yRaw < (cardinalMean['yN']-10)): #Use y. If EAST (usually 10uT less than NORTH)
				print "EAST"
				curDir = 1
			else:
				print "NORTH"
				curDir = 0
		elif (abs(xRaw-yRaw) > 30): #If SOUTH or WEST (both have very different x/y values)
			if (xRaw < (cardinalMean['xW']-10)): #Use x. If SOUTH (usually 10uT less than WEST)
				print "SOUTH"
				curDir = 2
			else:
				print "WEST"
				curDir = 3
		else: #default to the experimental estimation from August 4, 2016
			print "Direction?"
			if (abs(xRaw - yRaw) <= 10): #If cardinal direction is NORTH or EAST
	                	if ((xRaw>10) and (yRaw>10)):
	                        	print "NORTH"
	                	else:
	                        	print "EAST"
	        	else:#Cardinal direction is WEST or SOUTH
	                	if ((xRaw<-10) and (yRaw>10)):
	                        	print "SOUTH"
	                	else:
	                        	print "WEST"
	    
	    if (desDir != curDir):
			if ((curDir - desDir) < 0): #If we need to turn right, notice this is the default
				vehicle_servo.steer(50)
				time.sleep(0.5)
			else: #Turn left
				vehicle_servo.steer(120)
				time.sleep(0.5)
		else: #Stay centered
			vehicle_servo.center()
			time.sleep(0.5)

	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
