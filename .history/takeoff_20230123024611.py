import dronekit
import time
import socket
import math
import argparse

def connectCopter():
  parser = argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect', default="127.0.0.1:14550")
  args = parser.parse_args()
  
  str_connection = args.connect
  baud_rate = 57600
  vehicle = dronekit.connect(str_connection, baud=baud_rate, wait_ready=True)
  return vehicle

def arm_and_takeoff(aTargetAltitude):
  while vehicle.is_armable == False:
    print("Waiting for vehicle to become armable.")
    time.sleep(1)
  
  print("Vehicle is now armable")
  vehicle.mode = dronekit.VehicleMode("GUIDED")
  while vehicle.mode != "GUIDED":
    print("Waiting for vehicle to enter mode GUIDED")
    time.sleep(1)
    
  vehicle.armed = True
  while vehicle.armed == False:
    print("Waiting for drone become arming")
    time.sleep(1)

  vehicle.simple_takeoff(aTargetAltitude)
  while True:
    print("Vehicle is now armed.")
    print("Position: %s" %vehicle.location.global_relative_frame)
    print("Altitude: %s" %vehicle.attitude)
    print("Velocity: %s" %vehicle.velocity)
    print("Current Altitude: %s" %vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .95:
      break

    time.sleep(1)
    print("Look out..")
    return None

vehicle = connectCopter()
print("About to takeoff..")
vehicle.mode = dronekit.VehicleMode("GUIDED")
arm_and_takeoff(2)
vehicle.mode = dronekit.VehicleMode("LAND")
time.sleep()

print("End")
while True:
  time.sleep(1)

vehicle.close()