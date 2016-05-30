#calculate the the angle between two vectors

import math
import spidev
import time
import sys
import navio.util

from navio.mpu9250 import MPU9250

navio.util.check_apm()

imu=MPU9250()
if imu.testConnection():
	print "Connection established: ", imu.testConnection()
else:
	sys.exit("Connection established: False")

imu.initialize()

time.sleep(1)

while True:
	
	m9a, m9g, m9m = imu.getMotion9()

        if (m9m[1] == 0):
		m9m[1] = 0.0001

	x = int((m9m[0]) / (m9m[1]))

	radians = math.acos(x)
	theta = math.degrees(radians)

	if ((m9m[0] < 0) and (m9m[1] < 0)):
		angle = theta + 180
	elif ((m9m[0] < 0) and (m9m[1] > 0)):
		angle = theta + 180
	elif ((m9m[0] > 0) and (m9m[1] < 0)):
		angle = theta + 360
	else:
		angle = theta + 0
	
	print "Angle in degrees: ", angle
	print "Corrected angle in degrees: ", theta

	time.sleep(0.5)
