import dronekit
import time
import numpy as np
import plot

home_location = dronekit.LocationGlobalRelative(0, 0, 0)
distance_trajectory = 0
latitude_history = []
longitude_history = []
altitude_history = []


def kalman_filter(state, cov, u, z, A, B, H, Q, R):
    # State prediction
    state = np.dot(A, state) + np.dot(B, u)
    cov = np.dot(A, np.dot(cov, A.T)) + Q
    
    # Measurement update
    K = np.dot(cov, np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(cov, H.T)) + R)))
    state = state + np.dot(K, (z - np.dot(H, state)))
    cov = cov - np.dot(K, np.dot(H, cov))
    
    return state, cov

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
        
    print("Taking off!")
    u = np.array([[0.0], [0.0]])
    z = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon]])
    u[0,0] = vehicle.location.global_relative_frame.lat
    u[1,0] = vehicle.location.global_relative_frame.lon
    A = np.array([[1.0, 0.0], [0.0, 1.0]])
    B = np.array([[0.0, 0.0], [0.0, 0.0]])
    H = np.array([[1.0, 0.0], [0.0, 1.0]])
    Q = np.array([[0.1, 0.0], [0.0, 0.1]])
    R = np.array([[0.1, 0.0], [0.0, 0.1]])
    Ks = np.array([[1.0, 0.0], [0.0, 1.0]])
    cov = np.array([[1.0, 0.0], [0.0, 1.0]]) 
    
    while True:
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print('Reached target altitude: %s' % altitude)
            break
        
        # Run the kalman filter for estimate the position
        state, cov = kalman_filter(u, cov, u, z, A, B, H, Q, R)
        
        # Run the sliding mode conotrol to generate control inputs
        ref = np.array([
            [home_location.lat],
            [home_location.lon]
        ])
        u_control = sliding_mode_control(state, ref, Ks)
        
        print("Take Off Data : ")
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame)
        print([u_control[0,0], u_control[1,0]])
        print(vehicle.location.global_relative_frame)
        print("---------------------------------------------------------------------------")
        
        
        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_error = altitude - current_altitude
        throttle_value = int(1500 + altitude_error * 30)
        
        # Set the control channel override values
        vehicle.channels.overrides = {
            '1': int(1500 + u_control[0,0]),
            '2': int(1500 + u_control[1,0]),
            '3': throttle_value,
            '4': 1500
        }
        
        # Send the control inputs to the vehicle
        vehicle.flush()
        
        time.sleep(0.1)

def trajectory(vehicle, altitude, distance, duration, speed, scanning, plotting):
    global home_location
    global distance_trajectory
    distance_trajectory = distance
    
    u = np.array([[0.0], [0.0], [0.0], [0.0]])
    z = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon], [0.0], [0.0]])
    u[0,0] = vehicle.location.global_relative_frame.lat
    u[1,0] = vehicle.location.global_relative_frame.lon
    A = np.array([[1.0, 0.0, 0.1, 0.0], [0.0, 1.0, 0.0, 0.1], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    B = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
    H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    Q = np.array([[0.1, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0], [0.0, 0.0, 0.1, 0.0], [0.0, 0.0, 0.0, 0.1]])
    R = np.array([[0.1, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0], [0.0, 0.0, 0.1, 0.0], [0.0, 0.0, 0.0, 0.1]])
    Ks = np.array([[1.0, 0.0], [0.0, 1.0]])
    cov = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    t = 0  # Time in seconds
    u_control = np.array([[1500.0], [1500.0]])
    start_time = time.time()
    
    while True:
        # Compute the desired position on the circle
        ref = np.array([distance * np.cos(t), distance * np.sin(t)]).reshape(2,1) + np.array([[home_location.lat], [home_location.lon]])

        # Run the Kalman Filter to estimate the position and velocity
        state, cov = kalman_filter(u, cov, u_control, z, A, B, H, Q, R)

        # Run the Sliding Mode Control to generate control inputs
        u_control = sliding_mode_control(state[0:2,:], ref, Ks)

        # Set the Control Channel Override values
        # Set control altitude
        
        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_error = altitude - current_altitude
        throttle_value = int(1500 + altitude_error * speed)
        
        vehicle.channels.overrides = {
            '1': int(1500 + (u_control[0,0] * speed)), 
            '2': int(1500 + (u_control[1,0] * speed)), 
            '3': throttle_value, 
            '4': 1500
        }

        # Send the control inputs to the vehicle
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
    global distance_trajectory
    print('Descending to the ground...')
    u = np.array([[0.0], [0.0], [0.0], [0.0]])
    z = np.array([[vehicle.location.global_relative_frame.lat], [vehicle.location.global_relative_frame.lon], [0.0], [0.0]])
    u[0,0] = vehicle.location.global_relative_frame.lat
    u[1,0] = vehicle.location.global_relative_frame.lon
    A = np.array([[1.0, 0.0, 0.1, 0.0], [0.0, 1.0, 0.0, 0.1], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    B = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
    H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    Q = np.array([[0.1, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0], [0.0, 0.0, 0.1, 0.0], [0.0, 0.0, 0.0, 0.1]])
    R = np.array([[0.1, 0.0, 0.0, 0.0], [0.0, 0.1, 0.0, 0.0], [0.0, 0.0, 0.1, 0.0], [0.0, 0.0, 0.0, 0.1]])
    Ks = np.array([[1.0, 0.0], [0.0, 1.0]])
    cov = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    t = 0  # Time in seconds
    u_alt = 0.1
    u_control = np.array([[1500.0], [1500.0]])
    while True:
        # Compute the desired position on the circle
        ref = np.array([distance_trajectory * np.cos(t), distance_trajectory * np.sin(t)]).reshape(2,1) + np.array([[vehicle.home_location.lat], [vehicle.home_location.lon]])

        # Run the Kalman Filter to estimate the position and velocity
        state, cov = kalman_filter(u, cov, u_control, z, A, B, H, Q, R)

        # Run the Sliding Mode Control to generate control inputs
        u_control = sliding_mode_control(state[0:2,:], ref, Ks)

        # Set the Control Channel Override values
        
        current_altitude = vehicle.location.global_relative_frame.alt
        altitude_error = u_alt - current_altitude
        throttle_value = int(1500 + altitude_error * 35)
        
        vehicle.channels.overrides = {
            '1': int(1500 + u_control[0,0]), 
            '2': int(1500 + u_control[1,0]), 
            '3': throttle_value, 
            '4': 1500
        }

        # Send the control inputs to the vehicle
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


    if plotting: plot.show("smckf.png")
    # Disarm the vehicle
    vehicle.mode = dronekit.VehicleMode("LAND")
    print('Disarming...')
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)



    