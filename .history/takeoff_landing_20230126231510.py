import dronekit
import pykalman
import time

alt_target = 5
hovering_time = 60
vehicle = dronekit.connect('127.0.0.1:14551', wait_ready=True)
vehicle.simple_takeoff(alt_target)

# Inisialize example of kalman filter
kf = pykalman.KalmanFilter(transition_matrices=[[1, 1], [0, 1]],
                  observation_matrices=[[1, 0]],
                  initial_state_mean=[0, 0],
                  initial_state_covariance=[[0.1, 0], [0, 0.1]],
                  observation_covariance=[[0.1]])

# Isialize example of sliding controll mode 
def sliding_mode_control(vehicle, desired_altitude):
    current_altitude = vehicle.location.global_relative_frame.alt
    error = desired_altitude - current_altitude
    if abs(error) < 0.1:
        vehicle.channels.overrides = {'3': 1500}
    elif error > 0:
        vehicle.channels.overrides = {'3': 1600}
    else:
        vehicle.channels.overrides = {'3': 1400}