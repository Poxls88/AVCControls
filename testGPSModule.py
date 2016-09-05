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
		#print GPSfix
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


vehicle_esc.stop()
vehicle_esc.rest()
vehicle_servo.center()
GPSNavInit()
#Start the NAVposllh messages
time.sleep(1)
msg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
for x in range(0,10):
	ubl.bus.xfer2(CFGmsg8_NAVposllh_yes)
	#ubl.bus.xfer2(msg)
print "Start NAVposllh"

while(True):
	try:
		pos = ubl.GPSfetch()
		if (pos != None):
			print pos
		#time.sleep(0.1)

	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
