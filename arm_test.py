import dronekit
import time
import socket
import math
import argparse

def connectCopter():
  parser = argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect')
  args = parser.parse_args()
  str_connection = args.connect
  baudrate = 57600
  vehicle = dronekit.connect(str_connection, 
                             baud=baudrate, 
                             wait_ready=True)
  return vehicle

def arm():
  while vehicle.is_armable == False:
    print("Waiting for vehicle to become armable")
    time.sleep(1)
  
  print("Vehicle is now armable")
  vehicle.armed = True
  
  while vehicle.armed == False:
    print("Waiting for arming drone")
    time.sleep(1)
    
  print("Vehicle is now armed")
  print("Position: %s" %vehicle.localtion.global_relative_frame)
  print("Altitude: %s" %vehicle.attitude)
  print("Velocity: %s" %vehicle.velocity)
  print("Look out")
  return None

vehicle = connectCopter()
print("About to takeoff")
arm()

print("Position: %s" %vehicle.location.global_relative_frame)
print("Altitude: %s" %vehicle.location.global_relative_frame)
print("Velocity: %s" %vehicle.location.global_relative_frame)
print("End of script")