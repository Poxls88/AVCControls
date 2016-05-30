"""
MS5611 driver code is placed under the BSD license.
Copyright (c) 2014, Emlid Limited, www.emlid.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.
	* Neither the name of the Emlid Limited nor the names of its contributors
	may be used to endorse or promote products derived from this software
	without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import spidev
import time
import math

from navio.mpu9250 import MPU9250

imu = MPU9250()
print "Connection established: ", imu.testConnection()

imu.initialize()

time.sleep(1)

while True:
	imu.read_mag()
	if (imu.magnetometer_data[0] == 0):
		x = (imu.magnetometer_data[1]/(0.001))
	else:
		x = (imu.magnetometer_data[1]/imu.magnetometer_data[0])

	radians = math.acos(x)
        theta = math.degrees(radians)

        if ((imu.magnetometer_data[0] < 0) and (imu.magnetometer_data[1] < 0)):
                angle = theta + 180
        elif ((imu.magnetometer_data[0] < 0) and (imu.magnetometer_data[1] > 0)):
                angle = theta + 180
        elif ((imu.magnetometer_data[0] > 0) and (imu.magnetometer_data[1] < 0)):
                angle = theta + 360
        else:
                angle = theta + 0
        # print "Accelerometer: ", imu.accelerometer_data
        # print "Gyroscope:     ", imu.gyroscope_data
        # print "Temperature:   ", imu.temperature
        print "Magnetometer:  ", imu.magnetometer_data
        print "Angle in degrees: ", angle
	print "Corrected angle in degrees: ", theta

	time.sleep(0.5)
