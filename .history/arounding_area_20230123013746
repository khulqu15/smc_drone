from dronekit import connect, VehicleMode
from time import sleep

# Connect to the Pixhawk
vehicle = connect('/dev/serial0', wait_ready=True)

# Set the vehicle to "GUIDED" mode
vehicle.mode = VehicleMode("GUIDED")

# Define the target area
area_coordinates = [(47.3977419, 8.5455932), (47.3977419, 8.5479932), (47.3957419, 8.5479932), (47.3957419, 8.5455932)]

# Set the vehicle's starting position
vehicle.simple_goto(area_coordinates[0])

# Wait for the vehicle to reach the starting position
while not vehicle.location.global_relative_frame.lat == area_coordinates[0][0] and not vehicle.location.global_relative_frame.lon == area_coordinates[0][1]:
    sleep(1)

# Begin navigating around the area
for coord in area_coordinates[1:]:
    vehicle.simple_goto(coord)
    while not vehicle.location.global_relative_frame.lat == coord[0] and not vehicle.location.global_relative_frame.lon == coord[1]:
        sleep(1)

# Return the vehicle to its starting position
vehicle.simple_goto(area_coordinates[0])

# Close the connection to the Pixhawk
vehicle.close()
