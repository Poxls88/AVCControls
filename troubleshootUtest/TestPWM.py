import time
import VehiclePWMModule
vehicle_servo = VehiclePWMModule.vehiclePWM("servo")
vehicle_esc = VehiclePWMModule.vehiclePWM("esc")


while(True):	
	#vehicle_esc.stop()
	vehicle_esc.accel(1)#Forward
	time.sleep(1)
	vehicle_esc.accel(-10)
	time.sleep(1)
	
