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
- September 9, 2016. Created file.

Resources:
https://docs.emlid.com/navio/Navio-dev/read-gps-data/
https://shahriar.svbtle.com/importing-star-in-python
https://docs.emlid.com/navio/Navio-dev/mpu9250-imu/
https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf

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
def calibrateMagNorth():
	xSet = []
	ySet = []
	print 'Place North'
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		#log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xNorth = float(sum(xSet))/max(len(xSet),1)#Mean
	yNorth = float(sum(ySet))/max(len(ySet),1)
	print 'Place East'
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		#log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xEast = float(sum(xSet))/max(len(xSet),1)#Mean
	yEast = float(sum(ySet))/max(len(ySet),1)
	print 'Place South'
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		#log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xSouth = float(sum(xSet))/max(len(xSet),1)#Mean
	ySouth = float(sum(ySet))/max(len(ySet),1)
	print 'Place West'
	time.sleep(4)
	for x in range(10):
		imu.read_mag()
		xSet.append(imu.magnetometer_data[0])
		ySet.append(imu.magnetometer_data[1])
		#log_mag.info('%f,%f' %(xSet[x],ySet[x]))
		time.sleep(0.5)
	xWest = float(sum(xSet))/max(len(xSet),1)
	yWest = float(sum(ySet))/max(len(ySet),1)#Mean
	#Y holds similar values for NORTH and SOUTH
	#X holds similar values for EAST and WEST
	means = {'xN':xNorth, 'yN':yNorth, 'xE':xEast, 'yE':yEast, 'xS':xSouth, 'yS':ySouth, 'xW':xWest, 'yW':yWest}
	time.sleep(0.5)
	return means
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
magMean = calibrateMagNorth()
#These mean values are the coordinates in the center of all readings (zero in the adafruit datasheet)
yMean = (magMean["yN"] + magMean["yE"] + magMean["yS"] + magMean["yW"])/4 # #Y's values are most useful
xMean = (magMean["xN"] + magMean["xE"] + magMean["xS"] + magMean["xW"])/4

#Re-enable GPS Messages
commUblox(CFGmsg8_NAVposllh_yes)
#backupMsg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
#commUblox(backupMsg)
print 'End calibrate IMU & Re-enable GPS messages'
# ---- End Calibrate IMU & Re-enable GPS Messages----

#Know next location
#calculate next waypoint heading
#Set course to move towards next waypoint

while(True):
	try:
		pos = ubl.GPSfetch()
		if (pos != None):
			print pos
			if (pos['hAcc'] <= 2000000):#change to 10 for actual testing
				#Prepare coordinate variables in order to calculate bearing
				lat = pos['lat']
				lon = pos['lon']
				phi = lat*(math.pi/180)
				lam = lon*(math.pi/180)
				
				#Equirectangular distance Approximation
				#The original formula from online calculates clockwise so 0-180 is east and 0--180 is west
				#x = (lam2-lam)*math.cos((phi+phi2)/2)
				#y = (phi2-phi)
				#My own formula calculates counterclockwise so 0-180 is west and 0--180 is east
				x = (lam-lam2)*math.cos((phi+phi2)/2)
				y = (phi2-phi)
				d = rE*math.sqrt((x*x)+(y*y)) #Only use is for debugging
				#print 'distance', d
				
				#Forward Bearing from Equirectagular Approximation
				bearWPsign = math.atan2(x,y)*(180/math.pi)
				bearWP = bearWPsign%360 #removes the sign so counter clockwise 0-360
				print 'bearing wp', bearWP
				
				if ( (abs(pos['lat']-lat2) <= 0.01) and (abs(pos['lon']-lon2) <= 0.01) ):
					print 'waypoint!'
					vehicle_esc.stop()
				else:
					vehicle_esc.accel(1)

			else: #If we don't have good accuracy
				print 'Bad accuracy!'
				#Move forward and back slowly until established valuable horizontal accuraccy
				
		#time.sleep(0.1)

	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
