import dronekit
import serial
import time
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pickle

matplotlib.use('Agg')

# Connect to vehicle
# copter = dronekit.connect("/dev/ttyACM0", baud=57600, wait_ready=True)
copter = dronekit.connect("127.0.0.1:14551", baud=57600, wait_ready=True)

latitude_data = []
longitude_data = []
altitude_data = []
# lidar = serial.Serial("/dev/serial0")

# if lidar.isOpen() == False:
#     lidar.open()

# def read_lidar_data(type="distance"):
    # while True:
    # counter = lidar.in_waiting # count the number of bytes of the serial port
    # if counter > 8:
    #     bytes_serial = lidar.read(9) # read 9 bytes
    #     lidar.reset_input_buffer() # reset buffer

    #     if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
    #         distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
    #         strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
    #         temperature = bytes_serial[6] + bytes_serial[7]*256 # temp in next two bytes
    #         temperature = (temperature/8.0) - 256.0 # temp scaling and offset
    #         if type == "distance": return distance/100.0
    #         if type == "strength": return strength
    #         if type == "temperature": return temperature

def arm_and_takeoff(altitude):
  """Arms vehicle and fly to a target altitude

  Args:
      altitude (int): Target to takeoff
  """
  print("Arming motors")
  # copter.mode = dronekit.VehicleMode("GUIDED")
  copter.mode = dronekit.VehicleMode("ACRO")
  copter.armed = True
  
  while not copter.armed:
    print("Waiting for arming...")
    time.sleep(1)
    
  print("Taking off !")
  
  # Take off to target altitude
  # copter.simple_takeoff(altitude)
  
  # Wait until the vehicle reaches a safe height before processing the goto
  # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
  while True:
    current_altitude = copter.location.global_relative_frame.alt
    altitude_error = altitude - current_altitude
    throttle_value = int(1500 + altitude_error * 35)
    copter.channels.overrides = {
      "3": throttle_value
    }
    print("Default channel overrides:", copter.channels.overrides)

    print("Altitude: ", copter.location.global_relative_frame.alt)
    # distance = read_lidar_data()
    # print("Data lidar (distance): ", distance)
    print("Location: ", copter.location.global_relative_frame)
    pos = copter.location.global_relative_frame
    latitude_data.append(pos.lat)
    longitude_data.append(pos.lon)
    altitude_data.append(pos.alt)
    if copter.location.global_relative_frame.alt >= altitude * 0.95:
      print("Reached target altitude")
      break
    # time.sleep(1)
  
def sliding_control_mode(target_location, kp, ki, kd):
  """
    Implements the sliding control mode for wind trajectory.
  
  """  
  # Set the vehicle to acro flight mode
  current_location = copter.location.global_relative_frame
  error_latitude = target_location.lat - current_location.lat
  error_longitude = target_location.lon - current_location.lon
  error_altitude = target_location.alt - current_location.alt
  p = kp * error_latitude, kp * error_longitude, kp * error_altitude
  i = ki * error_latitude, ki * error_longitude, ki * error_altitude
  d = kd * error_altitude

  return p[0] + i[0] + d, p[1] + i[1] + d, p[2] + d
  
def landing():
  """
  Lands the vehicle.
  """
  # copter.mode = dronekit.VehicleMode("GUIDED")
  copter.mode = dronekit.VehicleMode("ACRO")

  # Decreasing altitude for smooth landing
  landing_altitude = 0.3
  copter.airspeed = 0
  copter.groundspeed = 0
  
  # landing_point = dronekit.LocationGlobalRelative(copter.location.global_relative_frame.lat, copter.location.global_relative_frame.lon, landing_altitude)
  # copter.simple_goto(landing_point) 
  
  while True:
    current_altitude = copter.location.global_relative_frame.alt
    altitude_error = landing_altitude - current_altitude
    throttle_value = int(1500 + altitude_error * 20)
    copter.channels.overrides = {
      "3": throttle_value
    }
    print("Decrease Altitude: ", copter.location.global_relative_frame.alt)
    # distance = read_lidar_data()
    # print("Data lidar (distance): ", distance)
    pos = copter.location.global_relative_frame
    latitude_data.append(pos.lat)
    longitude_data.append(pos.lon)
    altitude_data.append(pos.alt)
    if copter.location.global_relative_frame.alt <= landing_altitude * 0.95:
      print("Reached target altitude")
      break
    time.sleep(1)
    
  time.sleep(5)
  
  print("Landing the copter")
  copter.mode = dronekit.VehicleMode("LAND")


# Take off
altitude = 0.9
arm_and_takeoff(altitude)

# time.sleep(5)

copter.mode = dronekit.VehicleMode("ACRO")

# Implement the sliding control mode
# lat=-35.363262,lon=149.1652374,alt=1.577
# -7.276161, 112.793064
target_location = dronekit.LocationGlobalRelative(-7.276102, 112.793068, 0.8)
kp = 0.1
k = 10
ki = 0.01
kd = 0.05


actual_error = [target_location.lat - copter.location.global_relative_frame.lat,
                target_location.lon - copter.location.global_relative_frame.lon,
                target_location.alt - copter.location.global_relative_frame.alt]

# 0.0000005

control_frequency = 10 # Hz
control_duration = 15 # Second

start_time = time.time()
wind_disturbance = 0
previos_error = [0,0,0]
throttle = [35, 35]

print("Sliding control mode activated !")

# copter.channels.overrides['3'] = 1500  # set roll to center
# copter.channels.overrides['4'] = 1500  # set pitch to center
# copter.channels.overrides['1'] = 1500  # set throttle to minimum
# copter.channels.overrides['2'] = 1500  # set yaw to center

tolerance_error = [abs(actual_error[0]) * 0.1,
                   abs(actual_error[1]) * 0.1]

while True:
  error = [target_location.lat - copter.location.global_relative_frame.lat,
           target_location.lon - copter.location.global_relative_frame.lon,
           target_location.alt - copter.location.global_relative_frame.alt]
  
  # 0.0000000001
  # 0.0000005
  # 0.0004000
  
  # sliding_control_signal = sliding_control_mode(target_location, kp, ki, kd)
  # sliding_control_signal = (int(sliding_control_signal[0] - wind_disturbance),
  #                         int(sliding_control_signal[1] - wind_disturbance),
  #                         int(sliding_control_signal[2] - wind_disturbance))
  # current_location = copter.location.global_relative_frame
  # distance = current_location.g(target_location)
  # print("Distance: ", distance)
  
  s = (lambda x:  error + k / x * np.sign(error))(k)
  u = -k * np.sign(s)
  print("s: ", s)
  print("u: ", u)
  print("s[0]: ", u.item(0))
  print("s[1]: ", u.item(1))
  
  print("Error: ", error)
  print("Actual Error: ", actual_error)
  
  if abs(error[0]) < abs(actual_error[0]):
    print("Lat: Semakin Dekat")
  else:
    print("Lat: Semakin Jauh")
    # throttle[0] = -throttle[0]

  if abs(error[1]) < abs(actual_error[1]):
    print("Lon: Semakin Dekat")
  else:
    print("Lon: Semakin Jauh")
    # throttle[0] = -throttle[0]

  # exp_error = ["{:e}".format(error[0]),
  #              "{:e}".for]


  print("Tolerance lat : ",  tolerance_error[0])
  print("Tolerance lon : ",  tolerance_error[1])
  print("Test error conditon lat : ",  float(abs(error[0])) <= tolerance_error[0])
  print("Test error conditon lon : ",  float(abs(error[1])) <= tolerance_error[1])

  if float(abs(error[0])) <= tolerance_error[0] and float(abs(error[1])) <= tolerance_error[1]: # tolerance
    print("Arrived !")
    break

  current_altitude = copter.location.global_relative_frame.alt
  altitude_error = altitude - current_altitude
  throttle_value = int(1500 + altitude_error * 100)
  
  copter.channels.overrides = {
    "1": int(1500 + s.item(0) * throttle[0]),
    "2": int(1500 + s.item(1) * throttle[1]),
    "3": throttle_value
  }
  print("Throttle 1 : ", int(1500 + s.item(0) * throttle[0]))
  print("Throttle 2 : ", int(1500 + s.item(1) * throttle[1]))
  
  pos = copter.location.global_relative_frame
  latitude_data.append(pos.lat)
  longitude_data.append(pos.lon)
  altitude_data.append(pos.alt)
      
  previos_error = error
  
  # if copter.location.global_relative_frame.lat == target_location.lat:
  #   if copter.location.global_relative_frame.lon == target_location.lon:
  #     break
    
  elapsed_time = time.time() - start_time
  if elapsed_time >= control_duration:
    break

  time.sleep(0.1)
  
# Land
landing()

# Close copter object before exiting script
copter.close()

# Plot the data using matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("Latitude")
ax.set_ylabel("Longitude")
ax.set_zlabel("Altitude (m)")
ax.plot(latitude_data, longitude_data, altitude_data)
# plt.show()
plt.savefig("3d_plot.png")
print("3D plot downloaded !")
