import dronekit
import time

vehicle = dronekit.connect('/dev/ttyACM0', wait_ready=True)

def sliding_surface(current_position, desired_position):
    return current_position - desired_position

def control_law(sliding_surface):
    return -0.5 * sliding_surface

desired_position = [0, 0, 10]

while True:
    current_position = vehicle.location.global_relative_frame.__dict__
    sliding = sliding_surface(current_position, desired_position)
    control_signal = control_law(sliding)
    vehicle.channels.overrides = {'1': control_signal}
    time.sleep(0.1)