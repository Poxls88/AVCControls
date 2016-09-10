"""
Robotritons testing version of gps navigation.

Purpose: Use reliable GPS data to control vehicle speed and calculate waypoint heading.
Requirements: A vehicle with at least one speed controller and one servo, and one Ublox NEO-M8N Standard Precision GNSS Module. The python modules sys, time, spidev, math, navio.util, VehicleGPSModule, and VehiclePWMModule 
Use: Set a waypoint. Instantiate esc, servo, and ublox objects then use their included methods as well as those defined here in order to wait until usable GPS data is secured.
	The vehicle's wheels will move, so help the vehicle approach the waypoint. The wheels will stop once the vehicle's latitude and longitude are both within 0.001 of the waypoint.
	Instantiate objects for an esc using vehiclePWM("esc"), a servo using vehiclePWM("servo"), and a ublox using U_blox()

Updates:
- September 9, 2016. Merged the testGPSModule into better named GPSBasicNav.py. The old testGPSModule and the new GPSBasicNav.py now calculate desired heading to a waypoint.

- May 26, 2016. I was finally able to reproduce the error of GPSfetch not returning a message. It has nothing to do with rapidly setting and changing CFG-Messages,
	(although 1 second mus pass after a message is enabled/disabled) but instead is caused when accel() changes direction inside the GPSfetch loop. When two back-back
	accel() statements have different signed speeds then the internal time.sleep() is executed for a total of 1 second. In other words each GPSfetch attempt will occur
	with a period of 1 second. But GPSfetch requires many, many, attempts to sucessfully find a NAVposllh message, about 1130 attempts by my count, so it would take
	~18 minutes for GPSfix to work running at one execution/second.
	
	Maybe a solution is to immediately poll for that type of message? I made the fetchSpecial method to poll for the desired message and expedite the message receival.
	But no luck: it takes just as long to get a response as not polling. Looks like it will take more effort than it is worth.
	I Guess threading or ommiting time.sleep() is the only solution.

- May 25, 2016. Found accel() execution of time.sleep() as the SOURCE OF ERROR causing GPSfetch() to never get a valid message. The pauses in
	the accel commands cannot be run with the nav message updates because it will take ~1000 tries to get a message. Also, noticed a POTENTIAL ERROR
	that accel statements move the vehicle only as long as the pwm generator maintains the same state. Aka, changing accel states too quickly
	means that output won't transfer to vehicle motion. Either structure your accel statements well, or they need to be made into a threaded process.
	
- May 24, 2016. The VehicleGPSModule methods NavStatusMessage and Parse_ubx naturally return values in addition to text. I made GPSfetch to capitolize
	on this behavior. GPSfetch packages the repetative set of commands to buffer and check message for data from the Ublox. Also I fixed accel from
	entering an infinite loop; now calling accel() multiple times and passing different signed arguments works.

Resources:
https://docs.emlid.com/navio/Navio-dev/read-gps-data/
https://shahriar.svbtle.com/importing-star-in-python

Resources GPS:
http://www.movable-type.co.uk/scripts/latlong.html
"""
import sys
import spidev
import time
import math
import navio.util
import VehiclePWMModule
from VehicleGPSModule import *

ubl = U_blox()
#Define Ublox reset messages
CFGmsg8_NAVposllh_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xb9]#Disable Ublox from publishing a NAVposllh	
CFGmsg8_NAVstatus_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xc0]#Disable Ublox from publishing a NAVstatus
CFGmsg8_NAVstatus_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x01,0x00,0x14,0xc2]#Enable Ublox to publish a NAVstatus
CFGmsg8_NAVposllh_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x13,0xbb]#Enable Ublox to publish a NAVposllh
#Define servo and esc
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
#Define secondary waypoint for bearing approximation
lat2 = 32.871894
lon2 = -117.2450076
phi2 = lat2*(math.pi/180)
lam2 = lon2*(math.pi/180)
#Define earth's mean volumetric radius
rE = 6371.008 

def commUblox(msg):
	for x in range(0,10):
		ubl.bus.xfer2(msg)

def GPSNavInit():
	#reset the Ublox messages
	commUblox(CFGmsg8_NAVposllh_no)
	commUblox(CFGmsg8_NAVstatus_no)
	print 'all NAV stopped \n'

	#Enable NAVstatus messages
	commUblox(CFGmsg8_NAVstatus_yes)
	print 'NAVstatus started \n'

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
	#print 'NAVstatus stopped \n'
	
	#Wiggle weels to indicate done init
	vehicle_servo.steer(-35)
	time.sleep(0.5)
	vehicle_servo.steer(35)
	time.sleep(0.5)
	vehicle_servo.center()

#Know next location
#calculate next waypoint heading
#Set course to move towards next waypoint

# ----- Initialization -----
#Start with vehicle at rest
vehicle_esc.stop()
vehicle_esc.rest()
#vehicle_servo.center()

#Initialize using the method above
GPSNavInit()

#Start the NAVposllh messages
time.sleep(1)
commUblox(CFGmsg8_NAVposllh_yes)
print "Started NAVposllh"
#backupMsg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
#commUblox(backupMsg)
# ----- Initialization End -----

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
