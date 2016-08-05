"""
Robotritons current definition and initialization module for pwm outputs to steering servo and esc controller.
Based on the Emlid Servo.py example and many personal iterations.

Purpose: Manage the PCA9685 to generate PWM signals output to the Savox SC-1258TG Servo and Xerun SCT-Pro ESC.
Requirements: Adafruit I2C and PCA9685 PWM generator drivers.
Use: Connect the Savox Servo and Xerun Esc to the Navio+ servo rail. Include this module in any python script. Create an object from the "vehiclePWM" class. Control it using the available methods.

Updates:
- May 26, 2016. Discovered the mallest equivalent accel() values are (1)&(-5) and that all (-4),(-5),(-6) calculate the same 12bit output, so 
	changing the mapping might not make a differenc. Also, centered servo's are at about steer(81)
- May 24, 2016. Changed variable PreviousSpeed to a class attribute instead of an object variable. This corrects flow control in method accel()
- May 21, 2016. All methods now respond to keyboard interrupts by relaxing the servo!

Resources:
PCA9685 Datasheet= http://www.nxp.com/documents/data_sheet/PCA9685.pdf
PCA9685 Emlid Explanation= http://docs.emlid.com/navio/Navio-dev/servo-and-rgb-led/
RPI2 <-> PCA9685 Pinout= https://community.emlid.com/t/electronic-schematics-and-mechanical-dimensions/174
Two descriptions of linux system GPIO interface
=http://falsinsoft.blogspot.com/2012/11/access-gpio-from-linux-user-space.html
=http://elinux.org/RPi_GPIO_Code_Samples#sysfs

Resources ESC Specific:
http://www.societyofrobots.com/robotforum/index.php?topic=4299.0
http://robots.dacloughb.com/project-2/esc-calibration-programming/ (edited)
http://aeroquad.com/showthread.php?5502-Which-PWM-frequency-do-I-use-to-control-my-ESC (edited)

Resources Syntax Specific:
http://stackoverflow.com/questions/21402177/methods-to-link-common-class-attributes-between-objects-not-inheritance
http://stackoverflow.com/questions/68645/static-class-variables-in-python/68672#68672
"""
# ---- Includes ---- 
from navio.adafruit_pwm_servo_driver import PWM
import math
import time

import sys
import signal

import navio.gpio
import navio.util

navio.util.check_apm()
# ---- End Includes ---- 

class vehiclePWM:
	
	#Notice: these variables outside of a method are class attributes. Their values are shared by all object instances of this class.
	PreviousSpeed = 0
	
	#Notice: the variables inside the __init__ method are created privately for each object.
	def __init__(self,hdwr):
# ---- Setup PWM Generator ----
		self.pin = navio.gpio.Pin(27)#Enable OE pin output.
		self.pin.write(0)#Drive the PCA9685BS model HVQFN28's OE pin #20 (27 RPI2) to "LOW"

		self.frequency = 45.05 #This frequency prioritizes use of the ESC and plays well with the servo.
		self.period = 1.0/self.frequency
		"""
		Please use 45.05Hz to properly operate the ESC controller.
		Please use frequencies 50Hz OR 300Hz for this code to reliably operate the steering servo.
		Other intermittent frequencies like 60,90,100 may or may not work. There doesn't appear to be a pattern.
		"""

		self.PCA9685_DEFAULT_ADDRESS = 0x40#Adresses the PCA9685BS register controlling LED14_OFF_L 
		self.pwm = PWM(0x40, debug=False)#Send the PWM register address to be used by internal modules adafruit_pwm_servo_driver.py, adafruit_i2c.py, and ultimately smbus.py
		self.pwm.setPWMFreq(self.frequency)#Set output pulse frequency of the PCA9685.
# ---- End PWM Generator Setup ----

# ---- Set Separate PWM Constants ----
		if (hdwr == "servo"):
			self.servoInit()
			self.rest()
		elif (hdwr == "esc"):
			self.escInit()
			self.stop()
		else:
			sys.exit("Specify the hardware to use vehiclePWM")

	def servoInit(self):
		self.NAVIO_RCOUTPUT = 3
		self.PWM_MinWidth = 0.000685 #S. This calculation was updated on April 30, 2016. It is made large enough to work at 300Hz
		self.PWM_MaxWidth = 0.002675 #S. This calculation was updated on April 30, 2016. It is made small enough to work at 50Hz
		self.PWM_Range = self.PWM_MaxWidth - self.PWM_MinWidth #0.00199
		self.SERVO_Range = 180.0 #Define servo degrees of movement. Must be a float for correct calculations/operation

	def escInit(self):
		self.NAVIO_RCOUTPUT = 5
		self.PWM_MinWidth = 0.001158 #S. This was measured on May 1, 2016.
		self.PWM_MaxWidth = 0.0021 #Changed this on May 2, 2016 to get about 1.91ms output
		self.PWM_Range = self.PWM_MaxWidth - self.PWM_MinWidth
		self.PWM_Stop = 0.001700 #T = 0.0017 is the neutral, corresponding to about 1.5ms
		self.Motor_Range = 100.0 #Define motor max speed
# ---- End PWM Constants ----

	"""
		A 12bit digital I2C signal controls the PCA9685 which then prepares an
		analog PWM wave for the Savox SC-1258TG servo.
		The PWM signal's absolute width directly controls the servo's angle.
		Emperical testing on April 4, 2016 shows that in order to maintain a
		constant signal width, the value of the 12 bits written must change
		with respect to the operating frequency. The specific relation:

			12BitWritten = 12BitRange * (DesiredSignalWidth * OperatingFrequency)
	"""
# ---- Servo Outputs ----
	def steer(self, deg):
		"""
		deg = 50 to turn full right
		deg = 120 to turn full left
		deg = 81 to center
		"""
		PWM_Width = self.PWM_MinWidth + (self.PWM_Range * (deg/self.SERVO_Range))# Convert our 180 degree positions to PWM widths in seconds
		SERVO_move = math.trunc((4096.0 * PWM_Width * self.frequency) -1)#Convert PWM widths to 12 bits (a scale of 0-4095) to write to the PCA9685.
		try:
			self.pwm.setPWM(self.NAVIO_RCOUTPUT, 0, SERVO_move)
		except KeyboardInterrupt:
			self.rest()
			sys.exit()

	def center(self):
		PWM_Width = self.PWM_MinWidth + (self.PWM_Range * (81/self.SERVO_Range))		
		SERVO_move = math.trunc((4096.0 * PWM_Width * self.frequency) -1)
		try:
			self.pwm.setPWM(self.NAVIO_RCOUTPUT, 0, SERVO_move)
		except KeyboardInterrupt:
			self.rest()
			sys.exit()
	def rest(self):
		self.pwm.setPWM(self.NAVIO_RCOUTPUT, 0, 0)#0S. To rest servo so it doesn't hold a position, send an invalid PWM signal by sending an invalid 0-4096 bits.
#---- End Servo Outputs ----

#---- ESC Outputs ----
	def accel(self, speed):
		#print 'start: s%d, p%d' % (speed,vehiclePWM.PreviousSpeed)
		if (speed < 0):
			if (vehiclePWM.PreviousSpeed > 0):
				self.stop()
				time.sleep(0.5)
				#print 'firstReverse'
				self.reverse(speed)
				time.sleep(0.25)
				self.stop()
				time.sleep(0.25)
				self.reverse(speed)
			else:
				self.reverse(speed)
		else:
			if (vehiclePWM.PreviousSpeed <= 0):
				self.stop()
				time.sleep(0.5)
				self.forward(speed)
			else:
				self.forward(speed)
		vehiclePWM.PreviousSpeed = speed
		#print 'end: s%d, p%d' % (speed,vehiclePWM.PreviousSpeed)

	def forward(self,Motor_speed):
		"""
		PWM width 1.725ms minimum forward loaded
		PWM width 2.140ms maximum forward loaded
		"""
		PWM_Width = 0.001725 + (0.000415 * (Motor_speed/self.Motor_Range))
		#print 'PWM_Width %f' %PWM_Width
		Motor_move = math.trunc((4096.0 * PWM_Width * self.frequency) -1)
		#print 'Motor_move %f' %Motor_move
		self.pwm.setPWM(self.NAVIO_RCOUTPUT,0,Motor_move)

	def reverse(self,Motor_speed):
		"""
		PWM width 1.590ms minimum reverse loaded
		PWM width 1.300ms maximum reverse loaded
		"""
		PWM_Width = 0.00159 + (0.000290 * (Motor_speed/self.Motor_Range))
		#print 'PWM_Width %f' %PWM_Width
		Motor_move = math.trunc((4096.0 * PWM_Width * self.frequency) -1)
		#print 'Motor_move %f' %Motor_move		
		self.pwm.setPWM(self.NAVIO_RCOUTPUT,0,Motor_move)
		

	def stop(self):
		stop = math.trunc((4096.0 * self.PWM_Stop * self.frequency) -1)
		self.pwm.setPWM(self.NAVIO_RCOUTPUT,0,stop)#To stop the motor send a constant stop pulse
#---- End ESC Outputs ----
