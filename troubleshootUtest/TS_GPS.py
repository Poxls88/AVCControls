"""
Robotritons troubleshooting version for gps navigation.

Purpose: Use reliable GPS data to navigate the vehicle
Requirements: A vehicle with at least one speed controller and one servo, and one Ublox NEO-M8N Standard Precision GNSS Module. The python modules sys, time, GPSConfigBackEnd, and VehiclePWMModule 
Use: Instantiate esc, servo, and ublox objects then use their included methods as well as those defined here in order to wait until usable GPS data is secured.
	Instantiate objects for an esc using vehiclePWM("esc"), a servo using vehiclePWM("servo"), and a ublox using U_blox()

Resources:
https://docs.emlid.com/navio/Navio-dev/read-gps-data/
https://shahriar.svbtle.com/importing-star-in-python
"""
import sys
import time
from GPSConfigBackEnd import *
import VehiclePWMModule

#make ubl object
ubl = U_blox()

def comm(msg):
	for x in range(0,10):
		ubl.bus.xfer2(msg)

#reset the Ublox messages
CFGmsg8_NAVposllh_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xb9]#Disable Ublox from publishing a NAVposllh	
CFGmsg8_NAVstatus_no = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xc0]#Disable Ublox from publishing a NAVstatus
#enable the Ublox messages
CFGmsg8_NAVstatus_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x01,0x00,0x14,0xc2]#Enable Ublox to publish a NAVstatus
CFGmsg8_NAVposllh_yes = [0xb5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x01,0x00,0x13,0xbb]#Enable Ublox to publish a NAVposllh

vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")
#ubl.debug=True
x=1
while(True):
	try:
		if (x==1):
			vehicle_esc.stop()
			vehicle_servo.rest()
			vehicle_servo.center()
			comm(CFGmsg8_NAVstatus_no)
			comm(CFGmsg8_NAVposllh_no)
			comm(CFGmsg8_NAVstatus_yes)
			#sTime = time.time()
			a = 0
			while not(a):
				a = ubl.GPSfetch()
			print(a)
			print 'once'
			x=2
		if (x==2):
			comm(CFGmsg8_NAVstatus_no)
			vehicle_servo.steer(45)
			time.sleep(0.5)
			vehicle_servo.steer(105)
			time.sleep(0.5)
			vehicle_servo.center()
			#ubl.debug=True
			print 'twice'
			x=3
			z=1
		comm(CFGmsg8_NAVposllh_yes)
		b = 0
		while not(b):
			#vehicle_esc.accel(3)			
			b = ubl.GPSfetch(1)
			#vehicle_esc.accel(-6)
			z=z+1
		print 'z %d' % z

	except KeyboardInterrupt:
		vehicle_esc.stop()
		vehicle_servo.rest()
		sys.exit()
