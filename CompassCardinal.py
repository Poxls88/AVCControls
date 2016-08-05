"""
Robotritons troubleshooting version for magnetometer navigation.

Purpose: Use a magnetometer to calculate the vehicle's cardinal heading.
Requirements: An InvenSense MPU-9250. The python modules spidev, time, math, navio.util, and navio.mpu9250
Use: Place the vehicle facing north. Instantiate an imu object, initialize it, then call the read_mag() method to update the list of magnetometer_data.
	The direction of the front of the vehicle will be calculated.

Updates:
- August 4, 2016. Temporarily created CompassCheck.py in order to output raw magnometer readings to a CSV file, I then graphed those on my computer to look for
	patterns for each cardinal direction. From my findings, I have updated this CompassTrig.py script to output the cardinal direction of the vehicle's head.
	The calibrateUnit() function was also replaced with calibrateCardinalMean() which finds current magnetometer readings for the cardinal directions.
	My findings from CompassCheck.py were that our initial impression was wrong and the magnometer's x/y data do not have a simple relation on an x-y plot, so trig functions
	do not help. Instead, the x/y data raw values are fairly predictable and lie within different bands for each cardinal direction. A simple conditional
	check of x/y reveals the current band and by extension, cardinal direction N,S,E,W.
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
		else:
			print "NORTH"
	elif (abs(xRaw-yRaw) > 30): #If SOUTH or WEST (both have very different x/y values)
		if (xRaw < (cardinalMean['xW']-10)): #Use x. If SOUTH (usually 10uT less than WEST)
			print "SOUTH"
		else:
			print "WEST"
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
	time.sleep(0.5)
