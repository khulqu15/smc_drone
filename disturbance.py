from dronekit import connect, VehicleMode
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.error_sum += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.error_sum + self.kd * derivative
        self.prev_error = error
        return output


# Connect to the drone
vehicle = connect('COM3', baud=57600, wait_ready=True)

# Set the drone's mode to hover
vehicle.mode = VehicleMode("STABILIZE")

# Set the desired attitude
desired_roll = 0
desired_pitch = 0

# Create the PID controllers
roll_controller = PIDController(1, 0, 0)
pitch_controller = PIDController(1, 0, 0)

# Set the control loop frequency
frequency = 20  # Hz
dt = 1 / frequency

while True:
    # Get the current attitude
    roll = vehicle.attitude.roll
    pitch = vehicle.attitude.pitch

    # Calculate the control signal
    roll_control = roll_controller.update(desired_roll - roll, dt)
    pitch_control = pitch_controller.update(desired_pitch - pitch, dt)

    # Update the attitude
    vehicle.attitude.roll_angle = roll_control
    vehicle.attitude.pitch_angle = pitch_control

    # Sleep for the control loop period
    time.sleep(dt)