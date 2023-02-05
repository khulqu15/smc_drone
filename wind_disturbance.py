from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def goto(vehicle, lat, lon, alt, groundspeed):
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location, groundspeed=groundspeed)

def wait_for_armable(vehicle):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable")
        time.sleep(1)

def arm(vehicle):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

def wait_for_gps(vehicle):
    while not vehicle.gps_0.fix_type > 0:
        print("Waiting for GPS...:", vehicle.gps_0.fix_type)
        time.sleep(1)

def arm_and_takeoff(vehicle, altitude):
  print("Arming motors")
  vehicle.mode = VehicleMode("GUIDED")
  vehicle.armed = True
  
  while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)
    
  print("Taking off !")
  # Take off to target altitude
  vehicle.simple_takeoff(altitude)
  
  # Wait until the vehicle reaches a safe height before processing the goto
  # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
  while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    # distance = read_lidar_data()
    # print("Data lidar (distance): ", distance)
    pos = vehicle.location.global_relative_frame
    latitude_data.append(pos.lat)
    longitude_data.append(pos.lon)
    altitude_data.append(pos.alt)
    print("Location: ", vehicle.location.global_relative_frame)
    if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
      print("Reached target altitude")
      break
    time.sleep(1)

def wind_disturbance_trajectory(vehicle, target_location, groundspeed, Kp, Ki, Kd):
    error = np.array([target_location.lat - vehicle.location.global_relative_frame.lat,
                      target_location.lon - vehicle.location.global_relative_frame.lon,
                      target_location.alt - vehicle.location.global_relative_frame.alt])
    integral = np.array([0, 0, 0])
    derivative = np.array([0, 0, 0])
    prev_error = np.array([0, 0, 0])

    while np.linalg.norm(error) > 1:
        # integral += error
        derivative = error - prev_error
        prev_error = error
        control = Kp * error + Ki * integral + Kd * derivative
        vehicle.channels.overrides = {'1': int(control[0]), '2': int(control[1]), '3': int(control[2])}
        error = np.array([target_location.lat - vehicle.location.global_relative_frame.lat,                          target_location.lon - vehicle.location.global_relative_frame.lon,                          target_location.alt - vehicle.location.global_relative_frame.alt])
        time.sleep(0.1)

vehicle = connect("127.0.0.1:14551", baud=57600, wait_ready=True)
# vehicle = connect("/dev/ttyACM0", wait_ready=True)
kp = 0.5
ki = 0.1
kd = 0.01
groundspeed = 5
latitude_data = []
longitude_data = []
altitude_data = []
vehicle.location.time
wind_velocity_x = 0
wind_velocity_y = 0
# -35.363016, 149.165204
target_location = LocationGlobalRelative(-35.363016, 149.165204, 1.2)
wind_velocity = (float(wind_velocity_x), float(wind_velocity_y))

previous_error = (0, 0)
altitude = 1.5
wait_for_armable(vehicle)
arm_and_takeoff(vehicle, 1.5)
wait_for_gps(vehicle)
vehicle.mode = VehicleMode("ACRO")

start_location = vehicle.location.global_relative_frame

while True:
    current_location = vehicle.location.global_relative_frame
    
    altitude_error = altitude - current_location.alt
    latitude_error = start_location.lat - current_location.lat
    longitude_error = start_location.lon - current_location.lon
    
    throttle_value = int(1500 + altitude_error * 100)
    
    error = (target_location.lat - current_location.lat, target_location.lon - current_location.lon)
    error = (error[0] + wind_velocity[0], error[1] + wind_velocity[1])
    desired_velocity = (kp * error[0] + kd * (error[0] - previous_error[0]), kp * error[1] + kd * (error[1] - previous_error[1]))
    previous_error = error
    print("1 : ", int((1500 + desired_velocity[0] * 100)))
    print("2 : ", int(1500 + desired_velocity[1] * 100))
    print("3 : ", throttle_value)
    vehicle.channels.overrides = {
        "1": int((1500 + desired_velocity[0] * 100)),
        "2": int(1500 + desired_velocity[1] * 100),
        "3": throttle_value
    }
    time.sleep(0.05)
    
# wind_disturbance_trajectory(vehicle, target_location, groundspeed, kp, ki, kd)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("Latitude")
ax.set_ylabel("Longitude")
ax.set_zlabel("Altitude (m)")
ax.plot(latitude_data, longitude_data, altitude_data)
plt.show()
plt.savefig("3d_plot.png", format="png")

# PENS (Polite)
# -7.276541, 112.793828