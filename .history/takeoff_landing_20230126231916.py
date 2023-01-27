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
        
while True:
  sliding_mode_control(vehicle, alt_target)
  attitude = vehicle.attitude
  print("Roll: %f, Pitch: %f, Yaw: %f, Alt: %f" % (attitude.roll, attitude.pitch, attitude.yaw, vehicle.location.global_relative_frame.alt))
  if vehicle.location.global_relative_frame.alt >= 1.2*0.95:
      print("Reached altitude")
      break
  time.sleep(1)
  
time.sleep(10)  
  
# Fly in a circle with a radius of 5 meters
for i in range(360):
  print("Roll: %f, Pitch: %f, Yaw: %f, Alt: %f" % (attitude.roll, attitude.pitch, attitude.yaw, vehicle.location.global_relative_frame.alt))
  vehicle.simple_goto(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon + 0.0001, vehicle.location.global_relative_frame.alt)
  time.sleep(1)

vehicle.simple_goto(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon + 0.0001, 0.8)

time.sleep(5)

landing_alt = 0.5
descent_rate = 0.5
while current_alt > landing_alt:
    print("Roll: %f, Pitch: %f, Yaw: %f, Alt: %f" % (attitude.roll, attitude.pitch, attitude.yaw, vehicle.location.global_relative_frame.alt))
  
    current_alt = vehicle.location.global_relative_frame.alt
    if current_alt - descent_rate > landing_alt:
        vehicle.commands.down = descent_rate
        
    time.sleep(0.5)
    
vehicle.mode = dronekit.VehicleMode("LAND")

vehicle.armed = False

vehicle.close()
