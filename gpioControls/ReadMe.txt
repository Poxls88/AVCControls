The sripts in this directory monitor the RPI2's gpio pins for an event.
They need to be started in the background during startup by including the lines:

sudo nohup /home/pi/Desktop/AVCControls/gpioControls/gpio16.sh &>/dev/null &
sudo nohup /home/pi/Desktop/AVCControls/gpioControls/gpio12.sh &>/dev/null &

in the folder /etc/rc.local. 
'nohup' allows the program to continue after the shell ends and sends a HangUP command.
'&' escapes and indicates a file descriptor
> redirects output
'&' at the end runs the process in the background

Use:
gpio12.sh will monitor the RPI2 GPIO-12 pin which is connected to the
Navio+'s AUX signal pin. It's use is to start a python autonomous program.

gpio16.sh will monitor the RPI2 GPIO-16 pin which is connected to the
soldered wires on the bottom of the RPI2. It's use is to properly shutdown
the RPI2 by a pushbutton event.
