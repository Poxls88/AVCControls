#!/bin/bash
# Robotritons RPI2 shutdown button script
#
#Purpose: Safely power off the RPI2 by using the 'sudo shutdown' command. This avoids potential card corruption.
#Requirements: Bash, a startup script such as /etc/rc.local to execute this file in the background, available GPIO12 pin.
#Use: Any momentary NormallyOpen switch like a single pull, single throw button should be connected to GPIO12 and ground.
# A line must be added in a startup script to execute the program as a background daemon. For example,
#	sudo nohup /home/pi/Desktop/AVCControls/gpioControls/gpio12.sh &>/dev/null &
# Then click the button twice within any 2 second interval. The RPI2 will then shutdown.
#
#Resources:
#https://learn.sparkfun.com/tutorials/switch-basics
#http://stackoverflow.com/questions/4906579/how-to-use-bash-to-create-a-folder-if-it-doesnt-already-exist
#http://stackoverflow.com/questions/408975/compare-integer-in-bash-unary-operator-expected
#
#http://ryanstutorials.net/bash-scripting-tutorial/bash-if-statements.php
#http://stackoverflow.com/questions/2188199/how-to-use-double-or-single-bracket-parentheses-curly-braces
#http://stackoverflow.com/questions/4468824/command-not-found-when-attempting-integer-equality-in-bash
#
#

# export GPIO pin 12 and set as an input with pull-up
if [ ! -d /sys/class/gpio/gpio12 ]
then
 echo "12" > /sys/class/gpio/export
 sleep 1
fi
echo "in" > /sys/class/gpio/gpio12/direction
echo "high" > /sys/class/gpio/gpio12/direction

# Prepare timer
lastT=0
pressT=0
dur=1

while [ true ]
do
#now=$(date +%s) #Track current time
#echo "now" $now

# monitor GPIO pin 16 to go low.
if [ "$(cat /sys/class/gpio/gpio12/value)" == '0' ]
then
 lastT=$pressT
 pressT=$(date +%s) #Update time of most recent button press
 dur=$((pressT - lastT)) #Calculate time between pressed buttons
 #echo "last" $lastT
 #echo "press" $pressT
 #echo "dur" $dur
fi

#Shutdown if the button was pressed in quick succession between 1 and 3 seconds.
if [ "$dur" -gt 1 ] && [ "$dur" -lt 4 ]
then
 echo "System will shutdown"
 sudo shutdown -hP now
 exit 0
fi
sleep 0.5
done
