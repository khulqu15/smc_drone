import dronekit
import serial
import time

# Connect to vehicle
# copter = dronekit.connect("/dev/ttyACM0", baud=57600, wait_ready=True)
copter = dronekit.connect("127.0.0.1:14551", baud=57600, wait_ready=True)

def arm_and_takeoff(altitude):
  """Arms vehicle and fly to a target altitude

  Args:
      altitude (int): Target to takeoff
  """
  print("Arming motors")
  copter.mode = dronekit.VehicleMode("GUIDED")
  copter.armed = True
  
  while not copter.armed:
    print("Waiting for arming...")
    time.sleep(1)
    
  print("Taking off !")
  # Take off to target altitude
  copter.simple_takeoff(altitude)
  
  # Wait until the vehicle reaches a safe height before processing the goto
  # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
  while True:
    print("Altitude: ", copter.location.global_relative_frame.alt)
    if copter.location.global_relative_frame.alt >= altitude * 0.95:
      print("Reached target altitude")
      break
    time.sleep(1)
  
def sliding_control_mode(wind_speed, wind_direction):
  """Implements the sliding control mode for wind trajectory.
  
  Args:
    wind_speed (float): Variable for wind speed
    wind_direction (float): Variable for wind direction
  
  """
  # Set the vehicle to acro flight mode
  
  # Implement the sliding control algorithm here
  # ...
  
  print("Sliding control mode activated !")

def landing():
  """
  Lands the vehicle.
  """
  copter.mode = dronekit.VehicleMode("GUIDED")

  # Decreasing altitude for smooth landing
  landing_altitude = 0.3
  copter.airspeed = 0
  copter.groundspeed = 0
  
  landing_point = dronekit.LocationGlobalRelative(copter.location.global_relative_frame.lat, copter.location.global_relative_frame.lon, landing_altitude)
  copter.simple_goto(landing_point) 
  
  while True:
    print("Decrease Altitude: ", copter.location.global_relative_frame.alt)
    if copter.location.global_relative_frame.alt <= landing_altitude * 0.95:
      print("Reached target altitude")
      break
    time.sleep(1)
    
  time.sleep(5)
  
  print("Landing the copter")
  copter.mode = dronekit.VehicleMode("LAND")


# Take off
altitude = 1.5
arm_and_takeoff(altitude)

# # Implement the sliding control mode
# wind_speed = 10
# wind_direction = 90
# sliding_control_mode(wind_speed, wind_direction)

time.sleep(10)

# Land
landing()

# Close copter object before exiting script
copter.close()