import dronekit
import time
import pykalman

altitute_target = 1
vehicle = dronekit.connect('/dev/ttyACM0', wait_ready=True)
vehicle.armed = True
vehicle.simple_takeoff(altitute_target)

def sliding_mode_control(vehicle, desired_altitude):
  current_altitude = vehicle.
  
while True:
  sliding_mode_control(vehicle, altitute_target)