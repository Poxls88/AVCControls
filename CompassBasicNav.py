"""
Robotritons testing version of compass navigation.

Purpose: Use a magnetometer to reliably steer the vehicle.
Requirements: An InvenSense MPU-9250. The python modules logging, sys, spidev, time, math, navio.util, VehiclePWMModule, and navio.mpu9250_better
Use: Input a desired direction and the vehicle will try to turn itself that way. Place the vehicle facing north. Instantiate an imu object, then
	initialize it, then calibrate N,E,S,W, finally call the read_mag() method to update the list of magnetometer_data.
	The program will calculate the vehicles current heading and the bearing to the desired angle. The vehicle will steer towards the angle.

Updates:
- September 10, 2016. calibrateMagNorth() -> calibrateMag(). Updated so sweep vehicle through all angles, don't just hold at cardinal directions

- September 9, 2016. Attempted to add basic PID (instead of in VehiclePWMModule) for steering towards the bearing, by increasing the steering angle
	by 5 more than the bearing angle. Removed it because the wheels' friction prevents precise movement. In order to keep some control
	the old modular steering was reinstated.

- September 8, 2016. Cleaned compling bugs and tested inside finding that its basic ability to steer towards a degree works!
	Replaced print statements with output from the logging library. root_log prints to the console, data_log formats csv and prints to a file.
	In the 'Logging Setup' section, change the message levelname to control what types of log messages are printed

- September 5, 2016. Heading now measurable in degrees by incorporating arctan and declenation formulas based on adafruits datasheet. Also incorporated steering towards a degree direction.

- August 5, 2016. Created the file.

Resources:
https://docs.emlid.com/navio/Navio-dev/mpu9250-imu/
https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf
https://shahriar.svbtle.com/importing-star-in-python

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
handler_mag = logging.FileHandler('compassCheckData/magnetometerData.csv',mode='w')
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
target = 90

def calibrateMag():
	time.sleep(5) #5 Seconds before calibration begins
	
	log_root.info('calibrateMag')
	#Indicate start of calibration
	vehicle_servo.steer(35)
	time.sleep(0.5)
	vehicle_servo.steer(-35)
	time.sleep(0.5)
	vehicle_servo.center()
	
	#Capture about 1000 points for the whole sweep
	xSet = []
	ySet = []
	for x in xrange(600):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		if (x == 150):
			#Indicate 1/4 done with 1 steer
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
		elif (x == 300):
			log_root.info('1/2 calibrateMag')
			#Indicate 2/4 done with 2 steers
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
			time.sleep(0.5)
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
		elif (x == 450):
			#Indicate 3/4 done with 3 steers
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
			time.sleep(0.5)
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
			time.sleep(0.5)
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
		else:
			time.sleep(0.05)

	log_root.info('End calibrateMag')
	log_mag.info('End calibrateMag')
	#Indicate end of calibration
	vehicle_servo.steer(35)
	time.sleep(0.5)
	vehicle_servo.steer(-35)
	time.sleep(0.5)
	vehicle_servo.center()
	
	#Mean values are the coordinates in the center of all readings (zero in the adafruit datasheet)
	xMean = float(sum(xSet))/max(len(xSet),1)
	yMean = float(sum(ySet))/max(len(ySet),1)
	#Y holds similar values for NORTH and SOUTH
	#X holds similar values for EAST and WEST
	#If Y is inside a small threshold of its median the vehicle faces NORTH or SOUTH
	#Otherwise
	#	Y points NORTH and reads above its median, meaning the vehicle faces mostly WEST
	#	Y points SOUTH and reads below its median, meaning the vehicle faces mostly EAST
	return {'x':xMean,'y':yMean}

#Store our calibrated mean values
magMeans = calibrateMag() #Y's values are most useful

#Start vehicle stopped
vehicle_esc.stop()
vehicle_esc.rest()
vehicle_servo.rest()

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
		yCtrd = yRaw-magMeans['y']#Current readings minus the mean
		xCtrd = xRaw-magMeans['x']
		
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
		
		#Finally, useful Relative Bearings are <180 and include a sign to denote direction. Subtracting by 360 adds that sign.
		if (bearBasic>180):
			bearRel=bearBasic-360
		else:
			bearRel=bearBasic
		log_root.debug('bearRel: %f' % (bearRel))

		#If not heading in correct direction
		if (abs(bearRel)>8):
			if (bearRel > 0): #If bearing is to the right of target
				#Turn left
				if (bearRel < 45):
					vehicle_servo.steer(15)
					print '15'
				#elif (bearRel < 90):
				#	vehicle_servo.steer(25)
				#	print '25'
				else:
					vehicle_servo.steer(35)
					print '35'
			else: #If bearing is to the left of target
				#Turn right
				if (bearRel > -45):
					vehicle_servo.steer(-15)
					print '-15'
				#elif (bearRel > -90):
				#	vehicle_servo.steer(-25)
				#	print '-25'
				else:
					vehicle_servo.steer(-35)
					print '-35'
			#Convert bearing angle to possible steering angle
			#vehicle_servo.steer(bearRel*35/180) #steer(+-35) is largest value and bearRel is signed
		else:#Stay centered
			vehicle_servo.center()
			print 'center'
			time.sleep(0.05)
			
	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
