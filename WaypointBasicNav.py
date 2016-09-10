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
- September 10, 2016. Wrote structure for a single waypoint navigation. Shortened a few comments to improve readability.

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

"""
import sys
import time
import math
import spidev
import navio.util
import VehiclePWMModule
from VehicleGPSModule import *
from navio.mpu9250_better import MPU9250

navio.util.check_apm()

# ---- Define Methods ----
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
	#reset/stop the Ublox messages
	commUblox(CFGmsg8_NAVposllh_no)
	commUblox(CFGmsg8_NAVstatus_no)
	#Enable NAVstatus messages
	commUblox(CFGmsg8_NAVstatus_yes)
	#Wait until we have a confirmed GPS fix
	goodGPSfix = False
	while not (goodGPSfix):
		GPSfix = ubl.GPSfetch()
		#print GPSfix
		if (GPSfix):
			if((GPSfix['fStatus'] == 2) or (GPSfix['fStatus'] == 3) or (GPSfix['fStatus'] == 4)):
				goodGPSfix = True
	print 'goodFix \n'
	#After confirmed fix, disable Navstatus messages
	commUblox(CFGmsg8_NAVstatus_no)
	#Wiggle weels to indicate done init
	vehicle_servo.steer(45)
	time.sleep(0.5)
	vehicle_servo.steer(105)
	time.sleep(0.5)
	vehicle_servo.center()
	# --- End GPS Methods ---
	
	# --- IMU Methods ---
def calibrateMag():
	time.sleep(5) #5 Seconds before calibration begins
	
	#Indicate start of calibration
	vehicle_servo.steer(35)
	time.sleep(0.5)
	vehicle_servo.steer(-35)
	time.sleep(0.5)
	vehicle_servo.center()
	
	#Capture about 1000 points for the whole sweep
	xSet = []
	ySet = []
	for x in xrange(1000):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		if (x == 250):
			#Indicate 1/4 done with 1 steer
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
		elif (x == 500):
			#Indicate 2/4 done with 2 steers
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
			time.sleep(0.5)
			vehicle_servo.steer(35)
			time.sleep(0.5)
			vehicle_servo.center()
		elif (x == 750):
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

# ---- Waypoint ----
latW = 32.871894
lonW = -117.2450076
phiW = latW*(math.pi/180)
lamW = lonW*(math.pi/180)
rE = 6371.008 #Earth's mean volumetric radius
# ---- End Waypoint ----

# ---- Instantiate Critical Objects ----
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
ubl = U_blox()
imu = MPU9250()
print 'Connection established: %s' %(imu.testConnection())
# ---- End Instantiate Critical Objects ----

# ---- Initialize esc, servo, IMU & GPS ----
#Start with vehicle at rest
vehicle_esc.stop()
vehicle_esc.rest()
#Initialize IMU & GPS
vehicle_servo.rest()
imu.initialize()
GPSNavInit()
print 'End initialize IMU & GPS'
# ---- End Initialize IMU & GPS ----

# ---- Calibrate IMU & Re-enable GPS Messages----
#Begin calibrate IMU
magMeans = calibrateMag() #Mean values are the coordinates in the center of all readings (zero in the adafruit datasheet). Y's values are most useful
#Re-enable GPS Messages
commUblox(CFGmsg8_NAVposllh_yes)
#backupMsg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
#commUblox(backupMsg)
print 'End calibrate IMU & Re-enable GPS messages'
# ---- End Calibrate IMU & Re-enable GPS Messages----

#Know next location
#calculate next waypoint heading
#Set course to move towards next waypoint

timeout = 0
toSpeed = 0 #Stop speed
toAngle = 0

while(True):
	try:
		# -------------------------------------
		# ---- Read GPS To Update Location ----
		# -------------------------------------
		pos = ubl.GPSfetch()
		if (pos != None):
			#print pos
			if (pos['hAcc'] <= 2000000):#If GPS accurate (change to 10 for actual testing)
				#Prepare coordinate variables in order to calculate bearing
				lat = pos['lat']
				lon = pos['lon']
				phi = lat*(math.pi/180)
				lam = lon*(math.pi/180)
				
				#Calculate the Approximate Distance from waypoint using an Equirectangular map model
				'''
				The original formula from online calculates clockwise so 0<->+180 is EAST and 0<->-180 is WEST
				x = (lam2-lam)*math.cos((phi+phi2)/2)
				y = (phi2-phi)
				'''
				#My own formula calculates counterclockwise so 0<->+180 is WEST and 0<->-180 is EAST
				x = (lam-lam2)*math.cos((phi+phi2)/2)
				y = (phi2-phi)
				d = rE*math.sqrt((x*x)+(y*y)) #Only use is for debugging
				#print 'distance', d
				
				#Calculate the Forward Bearing from previous Equirectangular map model
				bearWPsign = math.atan2(x,y)*(180/math.pi)
				bearWP = bearWPsign%360 #removes the sign so counter clockwise 0<->360
				print 'bearing wp', bearWP
				
				#Reset GPS timeout since GPS received a usable message
				timeout = 0
				
			else: #GPS not accurate
				print 'Bad accuracy!'
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
		'''
		#Convert the heading to range from 0<->360, WEST=90, EAST=270
		headRad = headRadSign%math.pi #Good for debugging, but unecessary to calculate heading
		headDeg = headDegSign%360 #Good for debugging, but unecessary to calculate heading
		#print 'Radians heading from north: %f' % (headRad)
		#print 'Degrees heading from North: %f' %(headDeg)
		'''
		#Calculate the relative bearing counterclockwise.
		#Relative bearing is the angle between vehicle's heading and target. Magnetic bearing is the angle between the vehicle, target, and north.
		bearBasic = (bearWP-headDegSign)%360 #Vehicle direction becomes "0" degree and reorients the waypoint bearing around the perspective of the vehicle.
		#bearRel = (target-headDeg)%360. Has more roundoff error
		#bearRel = (headDeg-target)%360. Use target as the reference "0" degree and reorients the vehicle around the target's perspective.
		
		#Prepare relative bearing. Keep under <180 and include a sign to denote direction.
		if (bearBasic>180):
			bearRel=bearBasic-360 #Subtracting by 360 adds sign
		else:
			bearRel=bearBasic
		#print 'bearRel: %f' %(bearRel)
		# ---------------------------------------------------
		# ---- End Read Magnetometer To Control Steering ----
		# ---------------------------------------------------
		
		# --------------------------
		# ---- Control Movement ----
		# --------------------------
		#Always continue steering
		if (abs(bearRel)>8): #If not bearing in correct direction...
			if (bearRel > 0): #If bearing is to the right of waypoint, turn LEFT
				if (bearRel < 45):
					vehicle_servo.steer(15)
					#print '15'
				else:
					vehicle_servo.steer(35)
					#print '35'
			else: #If bearing is to the left of waypoint, turn RIGHT
				if (bearRel > -45):
					vehicle_servo.steer(-15)
					#print '-15'
				else:
					vehicle_servo.steer(-35)
					#print '-35'
			#Convert bearing angle to possible steering angle
			#vehicle_servo.steer(bearRel*35/180) #steer(+-35) is largest value and bearRel is signed
		else:#If bearing in correct direction, CENTERED
			vehicle_servo.center()
			#time.sleep(0.05)
		
		#Always control movement
		if (timeout < 200): #If GPS hasn't timed out...
			#If arrived at destination, STOP
			if ( (abs(pos['lat']-lat2) <= 0.01) and (abs(pos['lon']-lon2) <= 0.01) ):
				print 'waypoint!'
				vehicle_esc.stop()
				vehicle_esc.rest()
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
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
