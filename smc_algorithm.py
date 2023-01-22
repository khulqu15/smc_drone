import numpy as np

class SlidingModeController:
  
  def __init__(self, kp, kv, ka):
    self.kp = kp
    self.kv = kv
    self.ka = ka
    
  def control(self,
              desired_position,
              current_position,
              current_velocity):
    error_position = desired_position - current_position
    error_velocity = -current_velocity
    # Propotional control
    u_p = self.kp * error_position
    # Velocity control
    u_v = self.kv * error_velocity
    # Acceleration control
    u_a = self.ka * (-np.dot(error_position, error_velocity))
    # Combining all control
    u = u_p + u_v + u_a
    
    return u
  
# Instantiate the control
kp = 0.5
kv = 0.1
ka = 0.01
controller = SlidingModeController(kp, kv, ka)

# Test the controller
desired_position = np.array([1,2,3])
current_position = np.array([0.9, 1.9, 2.8])
current_velocity = np.array([0.1, 0.2, 0.3])

control_input = controller.control(desired_position, current_position, current_velocity)