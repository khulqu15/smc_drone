import numpy as np
from dronekit import connect, VehicleMode

# Connect to the vehicle
vehicle = connect('COM3', baud=57600, wait_ready=True)

# Set the initial state
x = np.array([[0], [0], [0], [0]])  # [x, x_dot, y, y_dot]
P = np.array([[1000, 0, 0, 0],
              [0, 1000, 0, 0],
              [0, 0, 1000, 0],
              [0, 0, 0, 1000]])

# Define the matrices for the prediction step
dt = 0.1  # Time step
A = np.array([[1, dt, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, dt],
              [0, 0, 0, 1]])
B = np.array([[(dt**2)/2, 0],
              [dt, 0],
              [0, (dt**2)/2],
              [0, dt]])
u = np.array([[vehicle.velocity[0]], [vehicle.velocity[1]]])  # Control input

# Define the matrices for the update step
H = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])
R = np.array([[1, 0],
              [0, 1]])

# Loop through the filter
while True:
    # Prediction step
    x = np.dot(A, x) + np.dot(B, u)
    P = np.dot(np.dot(A, P), np.transpose(A))

    # Update step
    Z = np.array([[vehicle.location.local_frame.north], [vehicle.location.local_frame.east]])  # Measurement
    y = Z - np.dot(H, x)
    S = np.dot(np.dot(H, P), np.transpose(H)) + R
    K = np.dot(np.dot(P, np.transpose(H)), np.linalg.inv(S))
    x = x + np.dot(K, y)
    P = P - np.dot(np.dot(K, H), P)

    # Print the updated state
    print(x)
