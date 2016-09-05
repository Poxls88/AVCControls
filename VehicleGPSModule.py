"""
Robotritons in-use module for gps communication. Based on tbe Emlid GPS.py example.

Purpose: Define classes to handle communications with the Ublox NEO-M8N Standard Precision GNSS Module and methods to handle data retrieval. 
Requirements: The python modules copy, Queue, spidev, math, struct, navio.util, and one one Ublox NEO-M8N Standard Precision GNSS Module.
Use: First make an object of class U_blox(). Initialize communication by sending an I2C poll request "self.bus.xfer2(msg)" or using
	the "enable_posllh(self)" method. Finally call GPSfetch() to probe the Ublox module for a message, then store its returned value for use.
	The remaining methods control the actual handling of a message and ultimately customize the functionality of GPSfetch().

Updates:
- May 26, 2016. Added a debug object variable "self.debug" which, when True, makes GPSfetch() print strings instead of returning values.
	Also defined new method "fetchSpecial" to test polling the GPS for more immediate message response.	It is accesible through GPSfetch()'s optional argument.
- May 25, 2016. Modified GPSfetch() to print nothing and instead return a valued dictionary.

Resources:
https://www.u-blox.com/sites/default/files/NEO-M8N-FW3_DataSheet_%28UBX-15031086%29.pdf
https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf
https://pythontips.com/2013/08/04/args-and-kwargs-in-python-explained/
http://www.binaryhexconverter.com/decimal-to-hex-converter
"""

import copy
import Queue
import spidev
import math
import struct
import navio.util

navio.util.check_apm()

waiting_header = 0
msg_class = 1
msg_id = 2
length = 3
payload = 4
checksum = 5

class U_blox_message:
	def __init__(self, msg_class = 0, msg_id = 0, msg_length = 0, msg_payload = []):
		self.msg_class = msg_class
		self.msg_id = msg_id 
		self.msg_length = msg_length
		self.msg_payload = msg_payload
	
	def clear(self):
		self.msg_class = 0
		self.msg_id = 0
		self.msg_length = 0
		self.msg_payload = []

class U_blox:

	def __init__(self):
		self.mess_queue = Queue.Queue()
		self.curr_mess = U_blox_message()
		self.bus = spidev.SpiDev()
		self.bus.open(0,0)
		self.state=0
		self.counter1=0
		self.chk_a=0
		self.chk_b=0
		self.accepted_chk_a=0
		self.accepted_chk_b=0
		self.debug=False

	def enable_posllh(self):
		msg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
		self.bus.xfer2(msg)
	
	def enable_posstatus(self):
		msg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0f, 0x49]
		self.bus.xfer2(msg)


	def scan_ubx(self, byte):
		if(self.state == waiting_header):
			self.result = [0,0,0,0,0,0,0,0,0]
			self.accepted = 0
			self.chk_a = 0
			self.chk_b = 0
			if((self.counter1 == 0) & (byte == 0xb5)):
				self.counter1 += 1
			elif((self.counter1 == 0) & (byte != 0xb5)):
				self.state = waiting_header
				self.counter1 = 0
			elif((self.counter1 == 1) & (byte == 0x62)):
				self.counter1 = 0
				self.state = msg_class
			elif((self.counter1 == 1) & (byte != 0x62)):
				self.counter1 = 0
				self.state = waiting_header
		elif(self.state == msg_class):
			self.chk_a = (self.chk_a + byte)%256
			self.chk_b = (self.chk_b + self.chk_a)%256
			self.curr_mess.msg_class = byte
			self.state = msg_id
		elif(self.state == msg_id):
			self.chk_a = (self.chk_a + byte)%256
			self.chk_b = (self.chk_b + self.chk_a)%256
			self.curr_mess.msg_id = byte
			self.state = length
		elif(self.state == length):
			if(self.counter1 == 0):
				self.chk_a = (self.chk_a + byte)%256
				self.chk_b = (self.chk_b + self.chk_a)%256
				self.counter1 += 1
				self.curr_mess.msg_length = byte
			elif(self.counter1 == 1):
				self.chk_a = (self.chk_a + byte)%256
				self.chk_b = (self.chk_b + self.chk_a)%256
				self.counter1 = 0
				self.curr_mess.msg_length = self.curr_mess.msg_length + 256*byte
				self.state = payload
		elif(self.state == payload):
			self.chk_a = (self.chk_a + byte)%256
			self.chk_b = (self.chk_b + self.chk_a)%256
			
			self.curr_mess.msg_payload.append(byte)
			if(self.counter1 < self.curr_mess.msg_length - 1):
				self.counter1 += 1
			else:
				self.counter1 = 0
				self.state = checksum

		elif(self.state == checksum):
			if(self.counter1 == 0):
				self.accepted_chk_a = byte
				self.counter1 += 1
			elif(self.counter1 == 1):
				self.accepted_chk_b = byte
				self.counter1 = 0
				self.state = waiting_header
				self.curr_mess.msg_length = 0
				if((self.chk_a == self.accepted_chk_a) & (self.chk_b == self.accepted_chk_b)):
					self.mess_queue.put(copy.deepcopy(self.curr_mess))
					self.curr_mess.clear()
				else:
					print("Error! Checksum doesn't match")

	def parse_ubx(self):
		curr_values = [0,0,0,0,0,0,0]
		curr_mess = self.mess_queue.get(False)
		
		#If the buffer held a NAVposllh message
		if((curr_mess.msg_class  == 0x01) & (curr_mess.msg_id == 0x02)):
			print "NAVposllh message"
			msg = NavPosllhMsg()
			curr_values = struct.unpack("<IiiiiII", str(bytearray(curr_mess.msg_payload)))
			msg.itow = curr_values[0]#Assign the current values into the msg object's parameters
			msg.lon = curr_values[1]
			msg.lat = curr_values[2]
			msg.heightEll = curr_values[3]
			msg.heightSea = curr_values[4]
			msg.horAcc = curr_values[5]
			msg.verAcc = curr_values[6]
			if (self.debug == True): return msg
			return msg.GPSPosition()
		
		#If the buffer held a NAVstatus message
		if((curr_mess.msg_class == 0x01) & (curr_mess.msg_id == 0x03)):
			print "NAVstatus message"
			msg = NavStatusMsg()
			msg.fixStatus = curr_mess.msg_payload[4]
			msg.fixOk = curr_mess.msg_payload[5]
			if (self.debug == True): return msg
			return msg.GPSStatus()
		'''
		if((curr_mess.msg_class == 0x06) & (curr_mess.msg_id == 0x00)):
			msg = "Found a CFG-PRT I/O message response"
			return msg
		
		if((curr_mess.msg_class == 0x06) & (curr_mess.msg_id == 0x01)):
			msg = "Found a CFG-MSG poll response"
			return msg
		'''
		return None

	#A GPS single communication method
	def GPSfetch(self,*args):
		if (args):
			return self.fetchSpecial()
		buffer = self.bus.xfer2([100])
		#print buffer
		#yes there is stuff in the buffer, but self.scan_ubx(byt) is never returning a valid message after NavStatus sucesfully runs
		#This problem only happens if you put in a time.sleep() while scanning, even a 0.1 second sleep screws it up.
		for byt in buffer:
			self.scan_ubx(byt)
			if(self.mess_queue.empty() != True):
				#print "message"
				data = self.parse_ubx()
				if (data != None):
					if(self.debug == True):
						print(data)
					else:
						return data
		return None

	def fetchSpecial(self):
		"""
		(PG 135)
		The UBX protocol is designed so that messages can be polled by sending the message required to the receiver
		but without a payload (or with just a single parameter that identifies the poll request). The receiver then
		responds with the same message with the payload populated
		"""
		#msg = [0xb5, 0x62, 0x01, 0x03, 0x10,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x6D] #Status poll?
		#msg = [0xb5, 0x62, 0x01, 0x02, 0x1c,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xA6] #Posllh poll?
		msg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0e, 0x47]
		buffer = self.bus.xfer2(msg)
		for byte in buffer:
			self.scan_ubx(byte)
			if(self.mess_queue.empty() != True):
				data = self.parse_ubx()
				if (data != None):
					if (self.debug == True):
						print(data)
					else:
						return data
		return None

class NavStatusMsg:

	def __init__(self):
		self.fixOk = 0
		self.fixStatus = 0

	def __str__(self):
		Status = "Reserved value. Current state unknown\n"
		if   (self.fixStatus == 0x00): Status = "no fix\n"
		elif (self.fixStatus == 0x01): Status = "dead reckoning only\n"
		elif (self.fixStatus == 0x02): Status = "2D-fix\n"
		elif (self.fixStatus == 0x03): Status = "3D-fix\n"
		elif (self.fixStatus == 0x04): Status = "GPS + dead reckoning combined\n"
		elif (self.fixStatus == 0x05): Status = "Time only fix\n"
		return 'Current GPS status:\ngpsFixOk: {}\ngps Fix status: {}'.format(self.fixOk & 0x01, Status)
		
	def GPSStatus(self):
		"""
		0 = no fix
		1 = dead reckoning only
		2 = 2D-fix
		3 = 3D-fix
		4 = GPS + dead reckoning combined
		5 = Time only fix
		"""
		status = {'fStatus':0,'fOk':0}
		status['fStatus'] = self.fixStatus
		status['fOk'] = self.fixOk
		return status

class NavPosllhMsg:

	def __init__(self):
		self.itow=0
		self.lon=0
		self.lat=0
		self.heightEll=0
		self.heightSea=0
		self.horAcc=0
		self.verAcc=0

	def __str__(self):
		itow = "GPS Millisecond Time of Week: %d s" % (self.itow/1000)
		lon = "Longitude: %.6f"  % (self.lon/10000000.0)
		lat = "Latitude: %.6f" % (self.lat/10000000.0)
		heightEll = "Height above Ellipsoid: %.3f m" % (self.heightEll/1000.0)
		heightSea = "Height above mean sea level: %.3f m" % (self.heightSea/1000.0)
		horAcc = "Horizontal Accuracy Estateimate: %.3f m" % (self.horAcc/1000.0)
		verAcc = "Vertical Accuracy Estateimate: %.3f m" % (self.verAcc/1000.0)
		return '{}\n{}\n{}\n{}\n{}\n{}\n{}\n'.format(itow, lon, lat, heightEll, heightSea, horAcc, verAcc)
		
	def GPSPosition(self):
		"""Prepares and returns a dictionary holding gps position accuracy, lat, lon, and height"""
		position = {'hAcc':0, 'lon':0, 'lat':0, 'hEll':0}
		position['hAcc'] = self.horAcc/1000.0
		position['lon'] = self.lon/10000000.0
		position['lat'] = self.lat/10000000.0
		position['hEll'] = self.heightEll/1000.0
		return position
