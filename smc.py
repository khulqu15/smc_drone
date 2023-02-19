import dronekit
import time
import numpy as np
import plot

home_location = dronekit.LocationGlobalRelative(0, 0, 0)

def sliding_mode_control(state, ref, Ks):
    e = ref - state
    u = np.dot(Ks, np.sign(e))
    
    return u

def arm_takeoff(vehicle, altitude, scanning, plotting):
    global home_location
    home_location = vehicle.location.global_relative_frame
    
    print("Pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dronekit.VehicleMode("ACRO")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    state = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon]])
    ref = np.array([[vehicle.home_location.lat], [vehicle.home_location.lon]])
    Ks = np.array([[1.0, 0.0], [0.0, 1.0]])

    print("Take off!")
    while True:
         # Get the current state of the vehicle
        state = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon]])

        # Run the Sliding Mode Control to generate control inputs
        u_control = sliding_mode_control(state, ref, Ks)
        
        print("Take Off Data : ")
        print(vehicle.location.global_relative_frame)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame)
        print("---------------------------------------------------------------------------")

        plot.save(vehicle.location.global_relative_frame)

        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_error = altitude - current_altitude
        throttle_value = int(1500 + altitude_error * 35)
        
        # Set the control channel override values
        vehicle.channels.overrides = {
            '1': int(1500 + u_control[0,0]),
            '2': int(1500 + u_control[1,0]),
            '3': throttle_value,
            '4': 1500
        }
        
        # Send the control inputs to the vehicle
        vehicle.flush()
        
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print('Reached target altitude: %s' % altitude)
            break
        
        time.sleep(0.1)

def trajectory(vehicle, altitude, distance, duration, scanning, plotting):
    print("Trajectory")
    t = 0
    Ks = np.array([[1.0, 0.0], [0.0, 1.0]])
    start_time = time.time()
    while True:
        # Compute the desired position on the circle
        ref = np.array([distance * np.cos(t), distance * np.sin(t)]).reshape(2,1) + np.array([[vehicle.home_location.lat], [vehicle.home_location.lon]])

        # Get the current state of the vehicle
        state = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon]])

        # Run the Sliding Mode Control to generate control inputs
        u_control = sliding_mode_control(state, ref, Ks)
        
        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_error = altitude - current_altitude
        throttle_value = int(1500 + altitude_error * 35)
        
        # Set the control channel override values
        vehicle.channels.overrides = {
            '1': int(1500 + (u_control[0,0] * 100)), 
            '2': int(1500 + (u_control[1,0] * 100)), 
            '3': throttle_value,
            '4': 1500
        }
        vehicle.flush()
        
        # Print the estimated position and velocity
        print("Trajectory Data : ")
        print('Latitude: %s, Longitude: %s' % (state[0,0], state[1,0]))
        print('Velocity in x-axis: %s, Velocity in y-axis: %s' % (state[2,0], state[3,0]))
        print([u_control[0,0], u_control[1,0]])
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame)
        print(vehicle.location.global_relative_frame)
        print("---------------------------------------------------------------------------")

        plot.save(vehicle.location.global_relative_frame)

        # Update the time
        t += 0.1

        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            break
        
        # Sleep for a short period of time
        time.sleep(0.1)
        
        
def landing_disarm(vehicle, scanning, plotting):
    print("Landing")
    Ks = np.array([[1.0, 0.0], [0.0, 1.0]])
    u_alt = 0.1
    while True:
        # Get the current state of the vehicle
        state = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon]])

        # Compute the reference position for landing
        ref = np.array([[vehicle.home_location.lat], [vehicle.home_location.lon], [0.0], [0.0]])

        # Run the Sliding Mode Control to generate control inputs
        u_control = sliding_mode_control(state, ref, Ks)

        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_error = u_alt - current_altitude
        throttle_value = int(1500 + altitude_error * 35)
        
        vehicle.channels.overrides = {
            '1': int(1500 + (u_control[0,0] * 100)), 
            '2': int(1500 + (u_control[1,0] * 100)), 
            '3': throttle_value, 
            '4': 1500
        }
        vehicle.flush()
        
        # Print the estimated position and velocity
        print("Landing Data : ")
        print('Latitude: %s, Longitude: %s' % (state[0,0], state[1,0]))
        print('Velocity in x-axis: %s, Velocity in y-axis: %s' % (state[2,0], state[3,0]))
        print([u_control[0,0], u_control[1,0]])
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame)
        print(vehicle.location.global_relative_frame)
        print("---------------------------------------------------------------------------")

        plot.save(vehicle.location.global_relative_frame)

        # Check if the vehicle has landed
        if vehicle.location.global_relative_frame.alt <= u_alt * 1.10:
            print('Reached target altitude: %s' % u_alt)
            break

        # Sleep for a short period of time
        time.sleep(0.1)

    if plotting: plot.show("smc.png")

    # Disarm the vehicle
    vehicle.mode = dronekit.VehicleMode("LAND")
    print('Disarming...')
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)

    