from dronekit import connect, VehicleMode
import time

# Connect to the SITL simulator
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Set the vehicle into guided mode
vehicle.mode = VehicleMode("GUIDED")

# Wait for the mode to change
while not vehicle.mode.name=='GUIDED':
    print("Waiting for mode change ...")
    time.sleep(1)
print("Mode: %s" % vehicle.mode.name)

# Takeoff to a certain altitude
vehicle.simple_takeoff(10)

# Wait for the vehicle to reach the target altitude
while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt>=10*0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

# Rotate the drone for 360 degree
for i in range(36):
    vehicle.heading = i*10
    print("Heading: ", vehicle.heading)
    time.sleep(5)

# Landing
vehicle.mode = VehicleMode("LAND")
print("Landing...")
