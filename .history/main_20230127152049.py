import dronekit
import time
import pykalman

altitute_target = 10
hovering_time = 60
# vehicle = dronekit.connect('/dev/ttyACM0', wait_ready=True)
vehicle = dronekit.connect('127.0.0.1:14551', wait_ready=True)
vehicle.armed = True
vehicle.simple_takeoff(altitute_target)

kf = pykalman.KalmanFilter(transition_matrices=[[1, 1], [0, 1]],
                  observation_matrices=[[1, 0]],
                  initial_state_mean=[0, 0],
                  initial_state_covariance=[[0.1, 0], [0, 0.1]],
                  observation_covariance=[[0.1]])

def sliding_mode_control(vehicle, desired_altitude):
    current_altitude = vehicle.location.global_relative_frame.alt
    error = desired_altitude - current_altitude
    if abs(error) < 0.1:
        vehicle.channels.overrides = {'3': 1500}
    elif error > 0:
        vehicle.channels.overrides = {'3': 1600}
    else:
        vehicle.channels.overrides = {'3': 1400}
  
while True:
  sliding_mode_control(vehicle, altitute_target)
  attitude = vehicle.attitude
  print("Roll: %f, Pitch: %f, Yaw: %f, Alt: %f" % (attitude.roll, attitude.pitch, attitude.yaw, vehicle.location.global_relative_frame.alt))
  if vehicle.location.global_relative_frame.alt >= 1.2*0.95:
      print("Reached altitude")
      break
  time.sleep(1)

time.sleep(hovering_time)

initial_pos = vehicle.location.global_relative_frame

while True:
    current_pos = vehicle.location.global_relative_frame
    # Use the Kalman filter to estimate the drone's position
    state_mean, state_cov = kf.filter_update(filtered_state_mean=initial_pos,
                                            filtered_state_covariance=kf.initial_state_covariance,
                                            observation=current_pos)
    # Update the initial position for the next iteration
    initial_pos = state_mean
    # Use the estimated position to control the drone's movement
    vehicle.simple_goto(state_mean)
    # Check if the drone has reached its destination
    if vehicle.location.global_relative_frame.alt >= 5:
        break

# Set the target altitude for the landing
landing_alt = 0.5
# Set the descent rate for the landing
descent_rate = 0.5
target_location = vehicle.LocationGlobalRelative(vehicle.location.lat, vehicle.location.lon, landing_alt)
# Begin the landing
vehicle.mode = dronekit.VehicleMode("LAND")

# Wait for the drone to reach the target altitude
while current_alt > landing_alt:
    # Get the current altitude of the drone
    current_alt = vehicle.location.global_relative_frame.alt
    
    # Check if the drone is descending at the desired rate
    if current_alt - descent_rate > landing_alt:
        # Command the drone to descend at the desired rate
        vehicle.commands.down = descent_rate
        
    # Wait for the next iteration
    time.sleep(0.5)

# Disarm the drone
vehicle.armed = False

vehicle.close()