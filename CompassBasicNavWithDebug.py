"""
Robotritons testing version of compass navigation.

Purpose: Use a magnetometer to reliably steer the vehicle.
Requirements: An InvenSense MPU-9250. The python modules logging, sys, spidev, time, math, navio.util, and navio.mpu9250
Use: Place the vehicle facing north. Instantiate an imu object, then initialize it, then calibrate N,E,S,W, finally call the read_mag() method to update the list of magnetometer_data
	and calculate the current direction.
	Input a desired direction and the vehicle will try to turn itself that way.

Updates:
- September 7, 2016. Replaced print statements with output from the logging library. root_log prints to the console, data_log formats csv and prints to a file.

- September 5, 2016. Heading now measurable in degrees by incorporating arctan and declenation formulas based on adafruits datasheet. Also incorporated steering towards a degree direction.

- August 5, 2016. Created the file.

Resources:
https://docs.emlid.com/navio/Navio-dev/mpu9250-imu/
https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf
https://shahriar.svbtle.com/importing-star-in-python
http://stackoverflow.com/questions/1621831/how-can-i-convert-coordinates-on-a-square-to-coordinates-on-a-circle

Resources Logging:
https://docs.python.org/2.7/howto/logging.html#advanced-logging-tutorial
https://docs.python.org/2.7/howto/logging-cookbook.html#logging-to-multiple-destinations
https://docs.python.org/2.7/library/logging.html#logger-objects

Resources Headings:
https://cdn-shop.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
https://docs.python.org/2/library/math.html (look for atan2())
http://aviation.stackexchange.com/questions/8000/what-are-the-differences-between-bearing-vs-course-vs-direction-vs-heading-vs-tr
http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf

Resources Bugs:
https://community.emlid.com/t/bug-magnetometer-reading-process-fail/688 #Community is aware of problem, but only C++ fixes are implemented
https://github.com/ArduPilot/ardupilot/pull/2487 #The immediate solution was to stop the AK8963 device reset. This is the solution I mimicked with "mpu9250_better"
https://github.com/ArduPilot/ardupilot/pull/2493 #Emlid developers have fixed the C++ driver
https://github.com/ArduPilot/ardupilot/pull/2504 #Someone else did it their own way for C++

#I don't think this is worth keeping.
		elif (yRaw >= yMean): #Otherwise Y points NORTH and reads above its median, thus the vehicle faces WEST
			yCtrd = yRaw-yMean #Current reading is translated so that it lies on a circle centered on the origin
			xCtrd = xRaw-xMean #Current readings minus the mean
			#If the vehicle faces WEST report 90 degrees from north
			#If the vehicle faces EAST report -90 degrees from north
			wSignedOff = math.atan2(xCtrd,yCtrd)*(180/math.pi)
			#If the vehicle faces WEST report 90 degrees from north
			#If the vehicle faces EAST reoprt 270 degreed from north
			wOff = wSignedOff%360
		elif (yRaw < yMean): #Otherwise Y points SOUTH and reads below its median, thus the vehicle faces EAST
			yCtrd = yRaw-yMean #Current reading is translated so that it lies on a circle centered on the origin
			xCtrd = xRaw-xMean #Current readings minus the mean
			#If the vehicle faces WEST report 90 degrees from north
			#If the vehicle faces EAST report -90 degrees from north
			eSignedOff = math.atan2(xCtrd,yCtrd)*(180/math.pi)
			#If the vehicle faces WEST report 90 degrees from north
			#If the vehicle faces EAST reoprt 270 degreed from north
			eOff = eSignedOff%360
		else:
			blah

"""

import logging
import sys
import spidev
import time
import math
import navio.util


from navio.mpu9250_better import MPU9250
import VehiclePWMModule

# ----- Setup Logging -----
#1) Loggers create log records. They are the outermost interface included in appliation code. root logger is default.
#2) Handlers send the log records to particular desitnations.
#3) Formatters specify the layout of the final output

#Add a basic console stream handler to the root (parent) logger
logging.basicConfig(level=logging.INFO)#Change debug level to control console output <----------
log_root = logging.getLogger('')#Assign an easy name to the root logger

#Create a separate logger for raw data from the magnetometer
log_mag = logging.getLogger('magnetometer')
log_mag.setLevel(logging.INFO) #Change debug level to control file output <----------

#Create a handler that outputs to a csv file
handler_mag = logging.FileHandler('MagnetometerData.csv')
handler_mag.setLevel(logging.DEBUG) #This handler handles all outputs to the file

#Create a formatter that labels the data
format_mag = logging.Formatter('%(levelname)-8s,%(message)s')

#Add format to handler, then handler to logger
handler_mag.setFormatter(format_mag)
log_mag.addHandler(handler_mag)

#Example in text logging call
#log_mag.info('This is some data %f', variable)
# ----- End Log Setup -----

navio.util.check_apm()
imu = MPU9250()
print "Connection established: ", imu.testConnection()

#initialize communication
imu.initialize()
time.sleep(1)
#initialize the servo and esc
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
#Initialize the angle from north to the target, this is the course angle.
target = 45

'''
desDir = 1 #0=N, 1=E, 2=S, 3=W
curDir = 0
'''

def calibrateMagNorth():
	xSet = []
	ySet = []
	log_root.info('Place North')
	time.sleep(2)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' % (xSet,ySet))
		time.sleep(0.5)
	xNorth = float(sum(xSet))/max(len(xSet),1)#Mean
	yNorth = float(sum(ySet))/max(len(ySet),1)
	
	log_root.info('Place East')
	time.sleep(2)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' % (xSet,ySet))
		time.sleep(0.5)
	xEast = float(sum(xSet))/max(len(xSet),1)#Mean
	yEast = float(sum(ySet))/max(len(ySet),1)

	log_root.info('Place South')
	time.sleep(2)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' % (xSet,ySet))
		time.sleep(0.5)
	xSouth = float(sum(xSet))/max(len(xSet),1)#Mean
	ySouth = float(sum(ySet))/max(len(ySet),1)

	log_root.info('Place West')
	time.sleep(2)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' % (xSet,ySet))
		time.sleep(0.5)
	xWest = float(sum(xSet))/max(len(xSet),1)
	yWest = float(sum(ySet))/max(len(ySet),1)#Mean
	
	#Y holds similar values for NORTH and SOUTH
	#X holds similar values for EAST and WEST
	means = {'xN':xNorth, 'yN':yNorth, 'xE':xEast, 'yE':yEast, 'xS':xSouth, 'yS':ySouth, 'xW':xWest, 'yW':yWest}
	time.sleep(0.5)
	return means

#Calculate our average direction values
magMean = calibrateMagNorth()
#These mean values are the coordinates in the center of all readings
yMean = (cardinalMean["yN"] + cardinalMean["yE"] + cardinalMean["yS"] + cardinalMean["yW"])/4
xMean = (cardinalMean["xN"] + cardinalMean["xE"] + cardinalMean["xS"] + cardinalMean["xW"])/4
#These median values (zero in the adafruit datasheet) could also be coordinates
yMedian = (cardinalMean["yN"] + cardinalMean["yS"])/2 #Y's values are most useful
xMedian = (cardinalMean["xE"] + cardinalMean["xW"])/2

#Start vehicle stopped
vehicle_esc.stop()
vehicle_esc.rest()
vehicle_servo.center()

while True:
	try:
		#Read our magnetometer
		#	Note: The magnetometer data is stored as a list ordered [x,y,z]
		#	Note: x+ is directed towards the front of the RPI2/Navio+ and y+ is directed towards the right of the RPI2/Navio+
		#	Note: all calculations assume x is the verticle axis and y is horizontal. Upsidedown vehicle reverses E<->W
		imu.read_mag()
		xRaw = imu.magnetometer_data[0] #print >> f, "X raw, %f" % (imu.magnetometer_data[0])
		yRaw = imu.magnetometer_data[1] #print >> f, "Y raw, %f" % (imu.magnetometer_data[1])
		log_mag.info('%f,%f' % (xRaw,yRaw))
		
		#Translate current reading so that it lies on a circle centered on the origin
		yCtrd = yRaw-yMean#Current readings minus the mean
		xCtrd = xRaw-xMean 
		
		#Calculate the heading counterclockwise. (angle between the vehicle and NORTH)
		#If the vehicle faces WEST report 90 degrees from north (1/2 pi)
		#If the vehicle faces EAST report -90 degrees from north (-1/2 pi)
		headRadSign = math.atan2(xCtrd,yCtrd)
		headDegSign = headRadSign*(180/math.pi)
		
		#Convert the heading to range from 0-360
		#If the vehicle faces WEST report 90 degrees from north (1/2 pi)
		#If the vehicle faces EAST reoprt 270 degrees from north (3/2 pi)
		headRad = headRadSign%math.pi
		headDeg = headDegSign%360
		log_root.info('Radians heading fron North: %f' % (headRad))
		log_root.info('Degrees heading fron North: %f' % (headDeg))
		
		if (abs(yRaw - yMean) < 1): #If Y is inside a small threshold of its median the vehicle faces NORTH or SOUTH:
			if (xRaw >= xMean): #If X reads above its median, the vehicle faces NORTH
				log_root.info('NORTH')
			elif (xRaw < xMean): #If X reads below its median, the vehicle faces SOUTH
				log_root.info('SOUTH')
			else:
				log_root.info('Direction? N/S')
			#Otherwise
			#Y points NORTH and reads above its median, meaning the vehicle faces mostly WEST
			#Y points SOUTH and reads below its median, meaning the vehicle faces mostly EAST

		if (abs(headDeg-target) > 10): #If not heading in correct direction
			#Find the angle between vehicle's heading and target, this is the Relative Bearing.
			#Equivalently this reorients the perspective so that the target is the "0" degree
			bearRel = (headDeg-target)%360
			if (bearRel <= 180): #If our current bearing offset is to the left of the target
				#Turn right
				vehicle_servo.steer(50)
			else: #Otherwise the bearing offset is to the right of the target
				#Turn left
				vehicle_servo.steer(120)
		else:#Stay centered
			vehicle_servo.center()
			
		'''	
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
			if (curDir < desDir): #If we need to turn right
				if ((desDir == 3) and (curDir == 0)): #Special case catch
					vehicle_servo.steer(120)
				else:
					vehicle_servo.steer(50)
			else: #Turn left
				if ((desDir == 0) and (curDir == 3)): #Special case catch
					vehicle_servo.steer(50)
				else:
					vehicle_servo.steer(120)
			time.sleep(0.5)
		else: #Stay centered
			vehicle_servo.center()
			time.sleep(0.5)
		'''

	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
