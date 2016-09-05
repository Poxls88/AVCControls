"""
Robotritons testing version of gps and compass navigation.

Purpose: Use reliable GPS data and compass direction navigate the vehicle
Requirements: A vehicle with at least one speed controller and one servo, and one Ublox NEO-M8N Standard Precision GNSS Module. An InvenSense MPU-9250. The python modules spidev, sys, time, math, navio.util, navio.mpu9250, VehicleGPSModule, and VehiclePWMModule 
Use: Instantiate esc, servo, and ublox objects then use their included methods as well as those defined here in order to wait until usable GPS data is secured.
	Instantiate objects for an esc using vehiclePWM("esc"), a servo using vehiclePWM("servo"), and a ublox using U_blox()

Updates:
- September 4, 2016. Created the file. Created GPSaccAdjust and GPSaccTimeSearch methods

	#Know next location
	destinations = [12343,224234324,3243242,42342343]
	waypoint = destination[i+1]
	
	#calculate next waypoint heading
	
	#Set course to move towards next waypoint
"""
#Import general
import sys
import spidev
import time
import math
import navio.util
import VehiclePWMModule
#Import for GPS
from VehicleGPSModule import *
#Import for compass
from navio.mpu9250_better import MPU9250

ubl = U_blox()
#Define Ublox reset messages
CFGmsg8_NAVposllh_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xb9]#Disable Ublox from publishing a NAVposllh	
CFGmsg8_NAVstatus_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xc0]#Disable Ublox from publishing a NAVstatus
CFGmsg8_NAVstatus_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x01,0x00,0x14,0xc2]#Enable Ublox to publish a NAVstatus
CFGmsg8_NAVposllh_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x13,0xbb]#Enable Ublox to publish a NAVposllh
#Define servo and esc
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")

def comm(msg):
	for x in range(0,10):
		ubl.bus.xfer2(msg)

def GPSNavInit():
	#reset the Ublox messages
	for x in range(0, 10):
		ubl.bus.xfer2(CFGmsg8_NAVposllh_no)
		ubl.bus.xfer2(CFGmsg8_NAVstatus_no)
	print 'all NAV stopped \n'

	#Enable NAVstatus messages
	for x in range(0, 10):
		ubl.bus.xfer2(CFGmsg8_NAVstatus_yes)
	print 'NAVstatus started \n'

	#Wait until we have a confirmed GPS fix
	goodGPSfix = False
	while not (goodGPSfix):
		GPSfix = ubl.GPSfetch()
		if (GPSfix):
			if((GPSfix['fStatus'] == 2) or (GPSfix['fStatus'] == 3) or (GPSfix['fStatus'] == 4)):
				goodGPSfix = True
	print 'goodFix \n'

	#After confirmed fix, disable Navstatus messages
	for x in range(0, 10):
		ubl.bus.xfer2(CFGmsg8_NAVstatus_no)
		
	#Wiggle weels to indicate done init
	vehicle_servo.steer(45)
	time.sleep(0.5)
	vehicle_servo.steer(105)
	time.sleep(0.5)
	vehicle_servo.center()

def GPSaccTimeSearch(duration):
	GPSaccStatus = False
	s = time.time()
	while((e-s) < duration):
			acc = ubl.GPSfetch()
			if (acc):
				if (acc['hAcc'] <= 200000):#change to 10 for actual testing
					GPSaccStatus = True
			e = time.time()
	#print 'passed'
	return GPSaccStatus

def GPSaccAdjust(): #assumes NAVposllh messages are already enabled	
	#Move forward and back slowly until established valuable horizontal accuraccy
	#ubl.debug=True
	print 'starting gps acc'
	i = 0
	while not (GPSaccTimeSearch(1)): #Start searching for a whole second
		if (i%2 == 0): #Alternate between move directions
			vehicle_esc.accel(1) #Move forward a little since it wasn't accurate
			#time.sleep(1)
		else:
			vehicle_esc.accel(-5) #Move backward to our original position
			#time.sleep(1)
		vehicle_esc.stop()
	print 'good acc \n'
	vehicle_esc.stop() #Stop when accurate gps

def NAVposllhUpdate():
	#Start the NAVposllh messages
	for x in range(0,10):
		ubl.bus.xfer2(CFGmsg8_NAVposllh_yes)
	##msg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
	
	#Grab the current gps location
	pos = ubl.GPSfetch()
	print pos, '\n'
	print pos['hAcc'], '\n'

vehicle_esc.stop()
vehicle_esc.rest()
vehicle_servo.center()
GPSNavInit()
GPSaccAdjust():
	
while(True):
	try:
		NAVposllhUpdate()

	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()

