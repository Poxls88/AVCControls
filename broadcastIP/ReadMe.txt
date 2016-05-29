This directory holds the smtp.py executable that is used to expidite
finding the IP address of the RaspberryPi once it is online.

Note that the file in Raspbian /etc/rc.local must be edited in order
to run the smtp.py executable at startup. The executable line should look like:

	python /home/pi/Desktop/AVCControls/broadcastIP/smtp.py

For more information about the process please refer to the web resource:
http://www.elinux.org/RPi_Email_IP_On_Boot_Debian

