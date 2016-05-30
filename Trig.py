#calculate the the angle between two vectors

import math
import argv

from AccelGyroMag import *

while True:

	x = (m9m[0]) / (m9m[1])

	radians = math.acos(x)
	theta = math.degrees(radians)

	if m9m[0] < 0 and m9m[1] < 0
		angle = theta + 180
	elif m9m[0] < 0 and m9m[1] > 0
		angle = theta + 180
	elif m9m[0] > 0 and m9m[1] < 0
		angle = theta + 360
	else:
		angle = theta + 0
	
	print "Angle in degrees: ", angle
	print "Corrected angle in degrees: ", theta

time.sleep(0.5)
