#!/bin/bash
 
# monitor GPIO pin 31 (wiringPi pin 1) for shutdown signal
 
# export GPIO pin 31 and set to input with pull-up
if [ ! -d /sys/class/gpio/gpio12 ]
then
 echo "12" > /sys/class/gpio/export
 sleep 1
fi

echo "in" > /sys/class/gpio/gpio12/direction
echo "high" > /sys/class/gpio/gpio12/direction
 
# wait for pin to go low
while [ true ]
do
if [ "$(cat /sys/class/gpio/gpio12/value)" == '0' ]
then
 echo "Raspberry Pi Shutting Down!"
 sudo shutdown -hP now
 exit 0
fi
sleep 1
done
