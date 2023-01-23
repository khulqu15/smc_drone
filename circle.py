from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the SITL instance
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Function to arm and then takeoff to a target altitude
def arm_and_takeoff(altitude):
    print("Arming and taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    vehicle.simple_takeoff(altitude)
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Function to circle a location
def circle_location(latitude, longitude, radius):
    print("Circling location...")
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(latitude, longitude, current_location.alt)
    vehicle.simple_goto(target_location, groundspeed=5)
    while True:
        if abs(vehicle.location.global_relative_frame.lat - target_location.lat) < radius and abs(vehicle.location.global_relative_frame.lon - target_location.lon) < radius:
            break
        time.sleep(1)

# Function to slide control mode landing
def slide_control_landing():
    print("Slide control landing...")
    vehicle.mode = VehicleMode("LAND")

# Arm and takeoff to 10m
arm_and_takeoff(10)

# Circle the location with radius of 0.5 degrees
circle_location(-35.362938, 149.165085, 0.5)

# Slide control mode landing
slide_control_landing()

# Close vehicle object before exiting script
vehicle.close()
