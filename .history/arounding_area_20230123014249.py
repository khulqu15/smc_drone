from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the Pixhawk using SITL
vehicle = connect('tcp:127.0.0.1:14550', wait_ready=True)

# Set the vehicle to "GUIDED" mode
vehicle.mode = VehicleMode("GUIDED")

# Define the target area
area_coordinates = [(-35.3632, 149.1652), (-35.3634, 149.1650), (-35.3630, 149.1648), (-35.3628, 149.1650)]

# Set the vehicle's starting position
vehicle.simple_goto(LocationGlobalRelative(area_coordinates[0][0], area_coordinates[0][1], 20))

# Wait for the vehicle to reach the starting position
while not vehicle.location.global_relative_frame.lat == area_coordinates[0][0] and not vehicle.location.global_relative_frame.lon == area_coordinates[0][1]:
    time.sleep(1)

# Begin navigating around the area
for coord in area_coordinates[1:]:
    vehicle.simple_goto(LocationGlobalRelative(coord[0], coord[1], 20))
    while not vehicle.location.global_relative_frame.lat == coord[0] and not vehicle.location.global_relative_frame.lon == coord[1]:
        time.sleep(1)

# Return the vehicle to its starting position
vehicle.simple_goto(LocationGlobalRelative(area_coordinates[0][0], area_coordinates[0][1], 20))

# Close the connection to the Pixhawk
vehicle.close()
