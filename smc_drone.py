import dronekit
import numpy as np
import time

class SlidingModeController:
  def __init__(self, kp, kv, ka, ks):
    self.kp = kp
    self.kv = kv
    self.ka = ka
    self.ks = ks
    
  def control(self, desired_position, desired_velocity, current_position, current_velocity):
    error_position = desired_position - current_position
    error_velocity = desired_velocity - current_velocity
    
    u_p = self.kp * error_position
    u_v = self.kv * error_velocity
    u_a = self.ka * (-np.dot(error_position, error_velocity))
    s = np.dot(error_position, error_velocity)
    u_s = self.ks * np.sign(s)
    
    u = u_p + u_v + u_a + u_s
    
    return u
  
vehicle = dronekit.connect("/dev/ttyACM0", baud=115200, wait_ready=True)
vehicle.armed = True
vehicle.simple_takeoff(2)

kp = 0.5
kv = 0.1
ka = 0.01
ks = 0.1
controller = SlidingModeController(kp, kv, ka, ks)

desired_position = np.array([vehicle.location.global_frame.lat,
                             vehicle.location.global_frame.lon,
                             vehicle.location.global_relative_frame.alt])
desired_velocity = np.array([0, 0, 0])

while True:
  current_position = np.array([vehicle.location.global_frame.lat,
                    vehicle.location.global_frame.lon,
                    vehicle.location.global_relative_frame.alt])
  current_velocity = np.array([vehicle.velocity[0],
                               vehicle.velocity[1],
                               vehicle.velocity[2]]) 
  control_input = controller.control(desired_position, desired_velocity, current_position, current_velocity)
  vehicle.attitude_target_local_ned = control_input
  time.sleep(0.1)
  
vehicle.close()