from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the SITL instance
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Function for takeoff
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Set the default sliding control mode
vehicle.parameters["SLIDE_SW"] = 1

# Takeoff to 10 meters
arm_and_takeoff(10)

# Fly in a circle with a radius of 10 meters
vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_relative_frame.lat,
                                           vehicle.location.global_relative_frame.lon,
                                           10),
                   10)
# Sleep for 10 seconds
time.sleep(10)

# Land
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()
