"""
Robotritons testing version of waypoint navigation.

Purpose: Vehicle will navigate itself to a single waypoint, using Magnetometer and GPS data, then the vehicle will stop.
Requirements: A vehicle with speed a controller, a servo, one InvenSense MPU-9250, and one Ublox NEO-M8N Standard Precision GNSS Module.
	The python modules sys, time, math, spidev, navio.util, VehicleGPSModule, VehiclePWMModule, and navio.mpu9250_better.
Use: Set a waypoint and the vehicle on the ground. In the code make sure to instantiate the esc, servo, ublox, and imu objects.
	Next, initialize the IMU and GPS. Finally, calibrate the imu and re-enable GPS position messages. The remaining loop will instruct the
	vehicle to move in the direction of the waypoint and stop once the vehicle's latitude and longitude are both within 0.001 of the waypoint.
	
	Example instantiate objects for an esc using motor=vehiclePWM("esc"), a servo using servo=vehiclePWM("servo"), a ublox using gps=U_blox(),
	and the magnetometer using imu=MPU9250()

Updates:
- September 10, 2016. Wrote structure for a single waypoint navigation. Shortened a few comments to improve readability. Included logging to
	console and file 'waypointData/waypointBasic.csv'. Added new exception handling.

- September 9, 2016. Created file.

Resources:
https://docs.emlid.com/navio/Navio-dev/read-gps-data/
https://shahriar.svbtle.com/importing-star-in-python
https://docs.emlid.com/navio/Navio-dev/mpu9250-imu/
https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf

Both GPS and IMU communicate through SPI: self.bus.open(spi_bus_number,spi_dev_number)
	spi_bus_number=0 for both
	spi_dev_number=0 for GPS, spi_dev_number=1 for IMU

Resources GPS:
http://www.movable-type.co.uk/scripts/latlong.html

Resources Bugs:
https://community.emlid.com/t/bug-magnetometer-reading-process-fail/688 #Community is aware of problem, but only C++ fixes are implemented
https://github.com/ArduPilot/ardupilot/pull/2487 #The immediate solution was to stop the AK8963 device reset. This is the solution I mimicked with "mpu9250_better"
https://github.com/ArduPilot/ardupilot/pull/2493 #Emlid developers have fixed the C++ driver
https://github.com/ArduPilot/ardupilot/pull/2504 #Someone else did it their own way for C++

Resources Logging:
https://docs.python.org/2.7/howto/logging.html#advanced-logging-tutorial
https://docs.python.org/2.7/howto/logging-cookbook.html#logging-to-multiple-destinations
https://docs.python.org/2.7/library/logging.html#logger-objects

Resources Exceptions And Traceback:
https://docs.python.org/2/library/exceptions.html#exceptions.TypeError
https://wiki.python.org/moin/HandlingExceptions
http://stackoverflow.com/questions/4564559/get-exception-description-and-stack-trace-which-caused-an-exception-all-as-a-st
"""
import logging
import traceback
import sys
import time
import math
import spidev
import navio.util
import VehiclePWMModule
from VehicleGPSModule import *
from navio.mpu9250_better import MPU9250

navio.util.check_apm()

# ----- Logging Setup -----
#1) Loggers create log records. They are the outermost interface included in appliation code. root logger is default.
#2) Handlers send the log records to particular desitnations.
#3) Formatters specify the layout of the final output

#All logging defaults to the root logger. Configure the default destination with root logger's built-in handler
logging.basicConfig(level=logging.DEBUG,format='%(levelname)-8s,%(message)s',filename='waypointData/waypointBasic.csv',filemode='w')
log_root = logging.getLogger('')#Assign an easy name to the root logger
#Create a separate handler output to the console stream. By default it's messages will propagate to the root ancestor.
handler_console = logging.StreamHandler()
handler_console.setLevel(logging.WARNING) #Change debug level to control console output <----------
#WARNING is default, INFO includes loop data, DEBUG includes calibration data too
#Create a formatter that labels the console data
format_console = logging.Formatter('%(levelname)-6s %(name)-6s %(message)s')
#Add format to handler, then handler to logger
handler_console.setFormatter(format_console)
log_root.addHandler(handler_console)
#Note: only loggers propagate messages to the root ancestor without considering the message levelname
#Different handlers for the same logger (root) will simply obey the higherarchy of levelnames to determine output(s)

#Example in text logging call
#log_root.info('This is some data %f' % variable)
# ----- End Loggin Setup -----

# ---- Waypoint ----
#EBU1 Loading Dock parkinglot south exit
latW = 32.882171
lonW = -117.235711

#Engineer Ln top of the "T"
#latW = 32.882281
#lonW = -117.235354

#Engineer Ln south end
#latW = 32.881772
#lonW = -117.234741

#EBU 1 Loading Dock street south end
#lat = 32.881373
#lon =-117.235912
#EBU 1 Back Patio

#latW = 32.871894
#lonW = -117.2350076
phiW = latW*(math.pi/180)
lamW = lonW*(math.pi/180)
rE = 6371.008 #Earth's mean volumetric radius
# ---- End Waypoint ----

# ---- Instantiate Critical Objects ----
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
ubl = U_blox()
imu = MPU9250()
log_root.warning('Connection established: %s' % imu.testConnection())
# ---- End Instantiate Critical Objects ----

# ---- Define Methods ----
	# --- Indication Methods ---
def wiggle(num, direction): #used particularly for visual calibration cues
	for times in range(num):
		vehicle_servo.steer(35*direction)
		time.sleep(0.5)
		vehicle_servo.center()
		time.sleep(0.5)
	# --- End Indication Methods ---

	# --- GPS Methods ---
#Define Ublox reset messages
CFGmsg8_NAVposllh_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xb9]#Disable Ublox from publishing a NAVposllh	
CFGmsg8_NAVstatus_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xc0]#Disable Ublox from publishing a NAVstatus
CFGmsg8_NAVstatus_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x01,0x00,0x14,0xc2]#Enable Ublox to publish a NAVstatus
CFGmsg8_NAVposllh_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x13,0xbb]#Enable Ublox to publish a NAVposllh

def commUblox(msg):
	for x in range(0,10):
		ubl.bus.xfer2(msg)

def GPSNavInit():
	log_root.warning('GPSNavInit')
	#reset/stop the Ublox messages
	commUblox(CFGmsg8_NAVposllh_no)
	commUblox(CFGmsg8_NAVstatus_no)
	#Enable NAVstatus messages
	commUblox(CFGmsg8_NAVstatus_yes)
	#Wait until we have a confirmed GPS fix
	goodGPSfix = False
	while not (goodGPSfix):
		GPSfix = ubl.GPSfetch()
		log_root.debug(GPSfix)
		if (GPSfix):
			if((GPSfix['fStatus'] == 2) or (GPSfix['fStatus'] == 3) or (GPSfix['fStatus'] == 4)):
				goodGPSfix = True
	#After confirmed fix, disable Navstatus messages
	commUblox(CFGmsg8_NAVstatus_no)
	log_root.warning('goodFix and end GPSNavInit')
	#Wiggle weels to indicate done init
	vehicle_servo.steer(-35)
	time.sleep(0.5)
	vehicle_servo.steer(35)
	time.sleep(0.5)
	vehicle_servo.center()
	# --- End GPS Methods ---
	
	# --- IMU Methods ---
def calibrateMag():
	time.sleep(4)
	#Indicate start of calibration
	log_root.warning('begin calibrateMag')
	wiggle(1,1)
	time.sleep(1) #5 seconds before calibration begins
	
	#Capture about 1000 points for the whole sweep
	xSet = []
	ySet = []
	for x in xrange(600):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		log_root.debug('%f,%f' %(xSet[x],ySet[x]))
		if (x == 150):
			#Indicate staring 2/4 with 2 wiggles
			#log_root.warning('begin 2/4 calibrateMag')
			wiggle(2,1)
		elif (x == 300):
			#Indicate starting 3/4 with 3 steers
			#log_root.warning('begin 3/4 calibrateMag')
			wiggle(3,1)
		elif (x == 450):
			#Indicate starting 4/4 with 4 steers
			#log_root.warning('begin 4/4 calibrateMag')
			wiggle(4,1)
		else:
			time.sleep(0.05)

	log_root.warning('end calibrateMag')
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
	# --- End IMU Methods ---
# ---- End Define Methods ----

# ---- Initialize esc, servo, IMU & GPS ----
try:
	#Start with vehicle at rest
	vehicle_esc.stop()
	vehicle_esc.rest()
	#Initialize IMU & GPS
	vehicle_servo.rest()
	imu.initialize()
	GPSNavInit()
	log_root.warning('End initialize IMU & GPS')
# ---- End Initialize IMU & GPS ----

# ---- Calibrate IMU & Re-enable GPS Messages----
	#Begin calibrate IMU
	magMeans = calibrateMag() #Mean values are the coordinates in the center of all readings (zero in the adafruit datasheet). Y's values are most useful
	#Re-enable GPS Messages
	commUblox(CFGmsg8_NAVposllh_yes)
	#backupMsg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
	#commUblox(backupMsg)
	log_root.warning('End calibrate IMU & Re-enable GPS messages')
except:
	log_root.warning('Abort@Calibrate')
	log_root.warning(traceback.format_exc())
	vehicle_esc.stop()
	vehicle_esc.rest()
	vehicle_servo.rest()
	sys.exit()
# ---- End Calibrate IMU & Re-enable GPS Messages----

#Know next location
#calculate next waypoint heading
#Set course to move towards next waypoint

timeout = 0
bearWP = 0
toSpeed = 0 #Stop speed
toAngle = 0
a = 0

try:
	log_root.warning('Begin try:while(True):')
	while(True):
		# -------------------------------------
		# ---- Read GPS To Update Location ----
		# -------------------------------------

		pos = ubl.GPSfetch()
		if (pos != None):
			log_root.warning('pos')
			log_root.debug(pos)
			if (pos['hAcc'] <= 10):#If GPS accurate (change to 10 for actual testing)
				#Prepare coordinate variables in order to calculate bearing
				lat = pos['lat']
				lon = pos['lon']
				phi = lat*(math.pi/180)
				lam = lon*(math.pi/180)
				
				#Calculate the Approximate Distance from waypoint using an Equirectangular map model
				'''
				The original formula from online calculates clockwise so 0<->+180 is EAST and 0<->-180 is WEST
				x = (lamW-lam)*math.cos((phi+phiW)/2)
				y = (phiW-phi)
				'''
				#My own formula calculates counterclockwise so 0<->+180 is WEST and 0<->-180 is EAST
				x = (lam-lamW)*math.cos((phi+phiW)/2)
				y = (phiW-phi)
				d = rE*math.sqrt((x*x)+(y*y)) #Only use is for debugging
				#print 'distance', d
				
				#Calculate the Forward Bearing from previous Equirectangular map model
				bearWPsign = math.atan2(x,y)*(180/math.pi)
				bearWP = bearWPsign%360 #removes the sign so counter clockwise 0<->360
				log_root.info('bearing wp %f' %bearWP)
				
				#Reset GPS timeout since GPS received a usable message
				timeout = 0
				
			else: #GPS not accurate
				log_root.warning('Bad accuracy!')
				#Timeout is not reset so vehicle will either: continue moving for a few more loops then stop <-or-> continue moving when horizontal accuraccy is good
		
		# -----------------------------------------
		# ---- End Read GPS To Update Location ----
		# -----------------------------------------
		
		# -----------------------------------------------
		# ---- Read Magnetometer To Control Steering ----
		# -----------------------------------------------
		#	Note: The magnetometer data is stored as a list ordered [x,y,z]
		#	Note: x+ is directed towards the front of the RPI2/Navio+ and y+ is directed towards the right of the RPI2/Navio+
		#	Note: all calculations assume x is the verticle axis and y is horizontal. Upsidedown vehicle reverses E<->W
		imu.read_mag()
		xRaw = imu.magnetometer_data[0]
		yRaw = imu.magnetometer_data[1]
		#print '%f,%f' % (xRaw,yRaw)
		
		#Translate current reading so that it lies on a circle centered on the origin
		yCtrd = yRaw-magMeans['y']#Current readings minus the mean
		xCtrd = xRaw-magMeans['x']
		
		#Calculate the heading counterclockwise 0<->+90(WEST) then -90<->0 (EAST). Heading is angle between the vehicle and NORTH.
		headRadSign = math.atan2(yCtrd,xCtrd) #atan2 in python takes (y, x). This is opposite to excel
		headDegSign = headRadSign*(180/math.pi)
		##log_root.info('headDegSign: %f' %headDegSign)

		'''
		#Convert the heading to range from 0<->360, WEST=90, EAST=270
		headRad = headRadSign%math.pi #Good for debugging, but unecessary to calculate heading
		headDeg = headDegSign%360 #Good for debugging, but unecessary to calculate heading
		#print 'Radians heading from north: %f' % (headRad)
		#print 'Degrees heading from North: %f' %(headDeg)
		'''
		#Calculate the relative bearing counterclockwise.
		#Relative bearing is the angle between vehicle's heading and target. Magnetic bearing is the angle between the vehicle, target, and north.
		bearBasic = (bearWP-headDegSign)%360 #The bearing is reoriented so that the waypoint is around the perspective of the vehicle. (vehicle direction "becomes the 0 degree")
		#bearRel = (target-headDeg)%360. Has more roundoff error
		#bearRel = (headDeg-target)%360. Use target as the reference "0" degree and reorients the vehicle around the target's perspective.
		##log_root.info('bearBasic: %f' %bearBasic)

		#Prepare relative bearing. Keep under <180 and include a sign to denote direction.
		if (bearBasic>180):
			bearRel=bearBasic-360 #Subtracting by 360 adds sign
		else:
			bearRel=bearBasic
		log_root.warning('bearRel: %f' %bearRel)
		# ---------------------------------------------------
		# ---- End Read Magnetometer To Control Steering ----
		# ---------------------------------------------------
		
		# --------------------------
		# ---- Control Movement ----
		# --------------------------
		#Always continue steering
		if (abs(bearRel)>8): #If not bearing in correct direction...
			if (bearRel > 0): #If bearing is to the right of waypoint, turn LEFT
				'''
				if (bearRel < 45):
					vehicle_servo.steer(20)
					#print '15'
				else:
					vehicle_servo.steer(35)
					#print '35'
				'''
				vehicle_servo.steer(35)
			else: #If bearing is to the left of waypoint, turn RIGHT
				'''if (bearRel > -45):
					vehicle_servo.steer(-20)
					#print '-15'
				else:
					vehicle_servo.steer(-35)
					#print '-35'
				'''
				vehicle_servo.steer(-35)
			#Convert bearing angle to possible steering angle
			#vehicle_servo.steer(bearRel*35/180) #steer(+-35) is largest value and bearRel is signed
		else:#If bearing in correct direction, CENTERED
			vehicle_servo.center()
			#time.sleep(0.05)
		
		#Always control movement
		if (timeout < 100): #If GPS hasn't timed out...
			#If arrived at destination, STOP
			if (pos != None):
				print pos['lat']
				if ( (abs(pos['lat']-latW) <= 0.0002) and (abs(pos['lon']-lonW) <= 0.0002) ):
					log_root.warning('Waypoint!')
					vehicle_esc.stop()
					vehicle_esc.rest()
					raise KeyboardInterrupt
				else: #Not arrived at destination, GO
					vehicle_esc.accel(1)
			timeout = timeout + 1 #Iterate timeout
		else: #GPS has timed out
			vehicle_esc.stop()
			vehicle_esc.rest()
		# ------------------------------
		# ---- End Control Movement ----
		# ------------------------------

except KeyboardInterrupt:
	log_root.warning('Abort@while(True): KeyboardInterrupt')
except TypeError:
	log_root.warning('Abort@while(True): TypeError')
finally:
	log_root.warning('Finally@while(True)')
	log_root.warning(traceback.format_exc())
	vehicle_esc.stop()
	vehicle_esc.rest()
	vehicle_servo.rest()
	sys.exit()
