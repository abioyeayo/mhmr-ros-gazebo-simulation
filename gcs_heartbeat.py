from pymavlink import mavutil
from time import sleep

# vehicle = mavutil.mavlink_connection('tcpin:localhost:4560')
vehicle = mavutil.mavlink_connection('127.0.0.1:14550', robust_parsing=True)
vehicle.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))
while True:
	vehicle.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
	sleep(0.05)
	
