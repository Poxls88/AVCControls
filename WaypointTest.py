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
# -------------------------
# ----- Logging Setup -----
# -------------------------
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
# ----- End Logging Setup -----
# -----------------------------

# ------------------
# ---- Waypoint ----
# ------------------
#EBU1 Loading Dock parkinglot south exit
#latW = 32.882171
#lonW = -117.235711

#Engineer Ln top of the "T"
#latW = 32.882281
#lonW = -117.235354

#Engineer Ln south end
#latW = 32.881772
#lonW = -117.234741

#EBU 1 Loading Dock street south end
latW = 32.881373
lonW =-117.235912
#EBU 1 Back Patio

#latW = 32.871894
#lonW = -117.2350076
phiW = latW*(math.pi/180)
lamW = lonW*(math.pi/180)
rE = 6371.008 #Earth's mean volumetric radius
# ---- End Waypoint ----
# ----------------------

# --------------------------------------
# ---- Instantiate Critical Objects ----
# --------------------------------------
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
ubl = U_blox()
imu = MPU9250()
log_root.warning('Connection established: %s' % imu.testConnection())
# ---- End Instantiate Critical Objects ----
# ------------------------------------------

# ------------------------
# ---- Define Methods ----
# ------------------------
	# --------------------------
	# --- Indication Methods ---
	# --------------------------
def wiggle(num, direction): #used particularly for visual calibration cues
	for times in range(num):
		vehicle_servo.steer(35*direction)
		time.sleep(0.5)
		vehicle_servo.center()
		time.sleep(0.5)
	# --- End Indication Methods ---
	# ------------------------------

	# -------------------
	# --- GPS Methods ---
	# -------------------
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

#magBearWPsign = 0
def GPSNavUpdate(): #MagneticBearing and location update
	pos = ubl.GPSfetch()
	if (pos != None):
		#After the GPS initialization it will take about 7000 loops of pos=ubl.GPSfetch so about 7 valid GPS (pos !=None) before data comes in consistantly
		log_root.warning('lat,lon')
		log_root.debug('%f,%f' %(pos['lat'],pos['lon']))
		if (pos['hAcc'] <= 10):#If GPS accurate (10 is ok)
			#Prepare coordinate variables in order to calculate magnetic bearing to waypoint
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
			
			#Calculate the Forward/Magnetic Bearing from previous Equirectangular map model.
			#Forward/Magnetic bearing is the angle from north to the waypoint with vehicle location as center.
			#Ex: waypoint to left and we are on right, so magnetic bearing is 90
			#Ex: waypoint to bottom right and we are on left, so magnetic bearing is 250
			magBearWPSign = math.atan2(x,y)*(180/math.pi) #N=0<->+180 is WEST and N=0<->-180 is EAST
			##magBearWP = bearWPsign%360 #removes the sign so counter clockwise 0<->360
			log_root.info('magnetic bearing signed to wp,%f' %magBearWPSign)
			#bearWP is changing by no more than 1.8 degrees even after the vehicle has moved 5 meters
			return [lat,lon,magBearWPSign,d] #magBearWP is later used to calculate RelativeBearing, current or target.
		else: #GPS not accurate. After multiple tests, i've not yet gotten a "bad accuracy" signal
			log_root.warning('Bad accuracy!')
			#Timeout is not reset so vehicle will either: continue moving for a few more loops then stop <-or-> continue moving when horizontal accuraccy is good
	return None
	# --- End GPS Methods ---
	# -----------------------
	
	# -------------------
	# --- IMU Methods ---
	# -------------------
def calibrateMag(auto=False):
	time.sleep(4)
	#Indicate start of calibration
	log_root.warning('begin calibrateMag')
	wiggle(1,1)
	time.sleep(1) #5 seconds before calibration begins
	if auto: #Read from file
		log_root.warning('auto')
		meansFile = open('waypointData/magnetometerMeans.txt','r')
		xMean = float(meansFile.readline().rstrip('\n'))
		yMean = float(meansFile.readline().rstrip('\n'))
		meansFile.close()
		#Internet help http://stackoverflow.com/questions/12330522/reading-a-file-without-newlines
		#calMeans = {'x':xMean,'y':yMean}
	else: #Standard manual calibration
		#Capture about 600 points for the whole sweep
		xSet = []
		ySet = []
		log_root.debug('xSet,ySet')
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
		meansFile = open('waypointData/magnetometerMeans.txt','w') #Open the file for writing
		meansFile.write('%f\n%f\n' % (xMean,yMean))#First line is xMean, second line is yMean
		meansFile.close()
		#calMeans = {'x':xMean,'y':yMean}
	log_root.warning('%f,%f' %(xMean,yMean))
	#calMeans #Returns the x,y means from manual or file calibration
	return {'x':xMean,'y':yMean}

magMeans = {'x':0,'y':0}
def updateMag():#current heading update
		#Note: The magnetometer data is stored as a list ordered [x,y,z]
		#Note: x+ is directed towards the front of the RPI2/Navio+ and y+ is directed towards the right of the RPI2/Navio+
		#Note: all calculations assume x is the verticle axis and y is horizontal. Upsidedown vehicle reverses E<->W
		imu.read_mag()
		xRaw = imu.magnetometer_data[0]
		yRaw = imu.magnetometer_data[1]
		
		#Translate current reading so that it lies on a circle centered on the origin
		yCtrd = yRaw-magMeans['y']#Current readings minus the mean
		xCtrd = xRaw-magMeans['x']
		
		#Calculate vehicle's current heading counterclockwise N=0<->+180(WEST) and N=0<->-180(EAST). Heading is angle between the vehicle and NORTH.
		headNoDecRadSign = math.atan2(yCtrd,xCtrd) #atan2 in python takes (y, x). This is opposite to excel
		headNoDecDegSign = headNoDecRadSign*(180/math.pi)
		##log_root.info('headDegSign: %f' %headDegSign)
		headDegSign = headNoDecDegSign + 11.7 #Adjust for declination (SD=11.7, boulder=8.2, avg = 10)
		
		#Convert the heading to range from 0<->360, WEST=90, EAST=270
		#headRad = headRadSign%math.pi #Good for debugging, but unecessary to calculate realative bearing
		##headDeg = headDegSign%360 #Good for debugging, but unecessary to calculate realative bearing
		log_root.debug('headDegSign,%f' %headDegSign)
		return headDegSign
	# --- End IMU Methods ---
	# -----------------------
# ---- End Define Methods ----
# ----------------------------

# ------------------------------------------
# ---- Initialize esc, servo, IMU & GPS ----
# ------------------------------------------
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
# ----------------------------------

# -----------------------------------------------
# ---- Calibrate IMU & Re-enable GPS Messages----
# -----------------------------------------------
	#Begin calibrate IMU. Pass 'file' argument to load averages from the last calibration
	#Mean values are the coordinates in the center of all readings (zero in the adafruit datasheet). Y's values are most useful
	magMeans = calibrateMag(auto=True) #change to auto to calibrate from file
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
	#meansFile.close() #not properly defined in global scope
	sys.exit()
# ---- End Calibrate IMU & Re-enable GPS Messages----
# ---------------------------------------------------

#Know next location
#calculate next waypoint heading
#Set course to move towards next waypoint
timeout = 0
toSpeed = 0 #Stop speed
toAngle = 0

try:
	log_root.warning('Begin try')
	log_root.warning('flush and average initial GPSNavUpdate() and updateMag()')
	#Flush out potentially inaccuration GPS readings
	x = 0
	while (True):
		flushPos = GPSNavUpdate()
		flushHead = updateMag()
		if (flushPos != None):
			x = x + 1
		if x == 8:
			break
	#Create an average to calculate inital waypoint bearing
	x = 0
	sumPos = [0,0,0,0]
	sumHead = 0
	while (True):
		onePos = GPSNavUpdate()
		oneHead = updateMag()
		if (onePos != None):
			sumPos[0] = sumPos[0] + onePos[0]
			sumPos[1] = sumPos[1] + onePos[1]
			sumPos[2] = sumPos[2] + onePos[2]
			sumPos[3] = sumPos[3] + onePos[3]
			sumHead = sumHead + sumHead
			x = x + 1
		if x == 5:
			break
	#Initial read GPS to receive an updated [lat,lon,magneticBearingSigned,distance]
	initPos = [0,0,0,0]
	initPos[0] = sumPos[0]/x
	initPos[1] = sumPos[1]/x
	initPos[2] = sumPos[2]/x
	initPos[3] = sumPos[3]/x
	#Initial read Magnetometer to receive an updated headingDegreesSigned
	initHead = sumHead/x
	log_root.warning('Begin update while(True)')
	
	init targetTime = 0
	steerAngle = 0
	while(True):
		#Constantly read GPS to receive an updated [lat,lon,magneticBearingSigned]
		curPos = GPSNavUpdate()
		#Constantly read Magnetometer to receive an updated headingDegreesSigned
		curHead = updateMag()
		#MagneticBearing and HeadingDegreesSigned are both necessary to calculate relative bearing
		
		#	Note:Relative bearing is the angle between vehicle's heading and the measured magneticBearing to the waypoint
		#Calculate InitialCurrentRelativeBearing as the angle between the current heading and the initial magneticBearing
		circleAlign = (initPos[2] - curHead)%360 #This just rotates the cirlce so that its 0<->360 aligns with the heading at "0". Aka the waypoint is around the perspective of the vehicle.
		if (circleAlign > 180): #Checking then subtracting by 360 gives negative sign to anything clockwise of our heading
			initRelBearSign = circleAlign - 360 #Subtracting by 
		else:
			initRelBearSign = circleAlign
		log_root.info('initRelBearSign, %f' %initRelBearSign)
		
		# ---- Continuous Steering ----
		
		lastAngle = steerAngle

		if (abs(initRelBearSign) > 8):
			if (initRelBearSign > 0): #If waypoint is counterclockwise then turn LEFT
				steerMax = 35
			else: #waypoint is clockwise turn RIGHT
				steerMax = -35
			steerAngle = initRelBearSign*35/180
		else:
			steerAngle = 0

		deltaSteer = abs(steerAngle) - abs(lastAngle)
		if (deltaSteer < 0): #Then lastAngle is bigger and we want to steer smaller
			compSteer = 0
		else:
			compSteer = steerMax
		
		if (time.time() < targetTime):
			vehicle_servo.steer(compSteer)
			#angleLast = compSteer
		else:
			vehicle_servo.steer(steerAngle)
			targetTime = time.time()+0.5
			
		#vehicle_servo.steer(initRelBearSign*35/180)#steer(+-35) is largest value and initRelBearSign is signed
		'''
		if (abs(relBear)>8): #If not bearing in correct direction. Gets rid of noise.
			if (relBear >0): #If waypoint is counterclockwise of heading, turn LEFT
				vehicle_servo.steer(35)
			else: #If waypoint is clockwise of heading, turn RIGHT
				vehicle_servo.steer(-35)
		else:#If bearing in correct direction, CENTERED
			vehicle_servo.center()
			#time.sleep(0.05)
		'''
		# ---- End Continuous Steering ----
		
		# ---- Waypoint Approach And Speed ----
		#lat smallest precision = 0.000008 (accurate and consistent within 8 ft)
		#lon smallest precision = 0.00002 (accurate and consistent within 8 ft)
		#lat/lon combined smallest precision = 0.00002,0.00002 (reliable geofence of 10ftx10ft square)
		#May need to adjust timeout period as precision threshold changes.
		if (curPos != None):
			if (curPos[3] <= .005):
				log_root.warning('Waypoint!')
				vehicle_esc.stop()
				vehicle_esc.rest()
				raise KeyboardInterrupt
			else: #Not arrived at destination...
				vehicle_esc.accel(1)
			timeout = 0 #Reset GPS timeout since GPS received a usable message
		elif (timeout < 150): #Otherwise check if GPS hasn't timed out, then GO
			vehicle_esc.accel(1)
		else: #But when GPS times out, STOP
			vehicle_esc.stop()
			vehicle_esc.rest()
		timeout = timeout + 1 #Iterate timeout
		'''
			if (abs(curPos[0]-latW) <= 0.00007): #0.00002
				latCur = True
			else:
				latCur = False
			if (abs(curPos[1]-lonW) <= 0.00007): #0.00002
				lonCur = True
			else:
				lonCur = False
			if ( latCur==True and lonCur==True): #If we've reached the geofenced waypoint
				log_root.warning('Waypoint!')
				vehicle_esc.stop()
				vehicle_esc.rest()
				raise KeyboardInterrupt
			else: #Not arrived at destination...
				vehicle_esc.accel(1)
			timeout = 0 #Reset GPS timeout since GPS received a usable message
		elif (timeout < 150): #Otherwise check if GPS hasn't timed out, then GO
			vehicle_esc.accel(1)
		else: #But when GPS times out, STOP
			vehicle_esc.stop()
			vehicle_esc.rest()
		timeout = timeout + 1 #Iterate timeout
		'''
		# ---- End Waypoint Approach And Speed ----
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
