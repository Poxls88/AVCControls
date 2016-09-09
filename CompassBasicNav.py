"""
Robotritons testing version of compass navigation.

Purpose: Use a magnetometer to reliably steer the vehicle.
Requirements: An InvenSense MPU-9250. The python modules logging, sys, spidev, time, math, navio.util, and navio.mpu9250
Use: Input a desired direction and the vehicle will try to turn itself that way. Place the vehicle facing north. Instantiate an imu object, then
	initialize it, then calibrate N,E,S,W, finally call the read_mag() method to update the list of magnetometer_data.
	The program will calculate the vehicles current heading and the bearing to the desired angle. The vehicle will steer towards the angle.

Updates:
- September 8, 2016. Cleaned compling bugs and tested inside finding that its basic ability to steer towards a degree works!
	Replaced print statements with output from the logging library. root_log prints to the console, data_log formats csv and prints to a file.
	In the 'Logging Setup' section, change the message levelname to control what types of log messages are printed

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

"""

import logging
import sys
import spidev
import time
import math
import navio.util

from navio.mpu9250_better import MPU9250
import VehiclePWMModule

# ----- Logging Setup -----
#1) Loggers create log records. They are the outermost interface included in appliation code. root logger is default.
#2) Handlers send the log records to particular desitnations.
#3) Formatters specify the layout of the final output

#Add a basic handler to the root (parent) logger. This handler's destination is the console stream
logging.basicConfig(level=logging.INFO,format='%(levelname)-6s %(name)-6s %(message)s')#Change debug level to control console output <----------
log_root = logging.getLogger('')#Assign an easy name to the root logger
#Create a separate logger for general output
#log_console = logging.getLogger('console')

#Create a separate logger for raw data from the magnetometer
log_mag = logging.getLogger('magnetometer')
log_mag.setLevel(logging.DEBUG) #Change debug level to control file output <----------

#Create a handler that outputs to a csv file
handler_mag = logging.FileHandler('CompassCheckData/magnetometerData.csv',mode='w')
handler_mag.setLevel(logging.DEBUG) #This handler handles all outputs to the file

#Create a formatter that labels the data
format_mag = logging.Formatter('%(levelname)-8s,%(message)s')

#Add format to handler, then handler to logger
handler_mag.setFormatter(format_mag)
log_mag.addHandler(handler_mag)

#Make sure the mag logger does not propagate messages to the root ancestor
#Propagation does not consider the message levelnames of the ancestor
#so we need to avoid flooding the console with mag logs of all levelnames
log_mag.propagate = False

#Example in text logging call
#log_mag.info('This is some data %f' % variable)
# ----- End Log Setup -----

navio.util.check_apm()
imu = MPU9250()
log_root.info('Connection established: %s' %(imu.testConnection()))
#print "Connection established: ", imu.testConnection()

#initialize communication
imu.initialize()
time.sleep(1)
#initialize the servo and esc
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
#Initialize the angle from north to the target, this is the course angle.
target = 270

def calibrateMagNorth():
	xSet = []
	ySet = []
	log_root.info('Place North')
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xNorth = float(sum(xSet))/max(len(xSet),1)#Mean
	yNorth = float(sum(ySet))/max(len(ySet),1)
	
	log_root.info('Place East')
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xEast = float(sum(xSet))/max(len(xSet),1)#Mean
	yEast = float(sum(ySet))/max(len(ySet),1)

	log_root.info('Place South')
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xSouth = float(sum(xSet))/max(len(xSet),1)#Mean
	ySouth = float(sum(ySet))/max(len(ySet),1)

	log_root.info('Place West')
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' %(xSet[x],ySet[x]))
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
yMean = (magMean["yN"] + magMean["yE"] + magMean["yS"] + magMean["yW"])/4
xMean = (magMean["xN"] + magMean["xE"] + magMean["xS"] + magMean["xW"])/4
#These median values (zero in the adafruit datasheet) could also be coordinates
yMedian = (magMean["yN"] + magMean["yS"])/2 #Y's values are most useful
xMedian = (magMean["xE"] + magMean["xW"])/2

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
		log_mag.debug('%f,%f' % (xRaw,yRaw))
		
		#Translate current reading so that it lies on a circle centered on the origin
		yCtrd = yRaw-yMean#Current readings minus the mean
		xCtrd = xRaw-xMean 
		
		#Calculate the heading counterclockwise. (angle between the vehicle and NORTH)
		#If the vehicle faces WEST report 90 degrees from north (1/2 pi)
		#If the vehicle faces EAST report -90 degrees from north (-1/2 pi)
		headRadSign = math.atan2(yCtrd,xCtrd) #atan2 in python takes (y, x). This is opposite to excel
		headDegSign = headRadSign*(180/math.pi)
		
		#Convert the heading to range from 0-360
		#If the vehicle faces WEST report 90 degrees from north (1/2 pi)
		#If the vehicle faces EAST reoprt 270 degrees from north (3/2 pi)
		headRad = headRadSign%math.pi #Good for debugging, but unecessary to calculate heading
		headDeg = headDegSign%360 #Good for debugging, but unecessary to calculate heading
		#print 'Radians heading from north: %f' % (headRad)
		log_root.debug('Degrees heading from North: %f' %(headDeg))
		
		#Find the ccw angle between vehicle's heading and target, this is the Relative Bearing.
		#This uses our vehicle as the reference "0" degree and equivalently reorients the target around the perspective of the vehicle
		bearBasic = (target-headDegSign)%360
		#bearRel = (target-headDeg)%360. Has more roundoff error
		#bearRel = (headDeg-target)%360. Uses target as the reference "0" degree and equivalently reorients the vehicle around the target's perspective.
		#Also, the angle between the target and north is the Magnetic Heading.
		
		#Finally useful Relative Bearings are <180 and include a sign to denote direction. Subtracting by 360 adds that sign.
		if (bearBasic>180):
			bearRel=bearBasic-360
		else:
			bearRel=bearBasic
		log_root.debug('bearRel: %f' % (bearRel))
		'''
		if (abs(yRaw - yMean) < 5): #If Y is inside a small threshold of its median the vehicle faces NORTH or SOUTH:
			if (xRaw >= xMean): #If X reads above its median, the vehicle faces NORTH
				log_root.info('NORTH')
			elif (xRaw < xMean): #If X reads below its median, the vehicle faces SOUTH
				log_root.info('SOUTH')
			else:
				log_root.info('Direction? N/S')
			#Otherwise
			#Y points NORTH and reads above its median, meaning the vehicle faces mostly WEST
			#Y points SOUTH and reads below its median, meaning the vehicle faces mostly EAST
		'''
		if (abs(bearRel)>10): #If not heading in correct direction
			if (bearRel > 0): #If our current bearing offset is to the right of the target
				#Turn left
				vehicle_servo.steer(120)
			else: #Otherwise the bearing offset is to the left of the target
				#Turn right
				vehicle_servo.steer(50)
		else:#Stay centered
			vehicle_servo.center()
			time.sleep(0.05)
			
	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
