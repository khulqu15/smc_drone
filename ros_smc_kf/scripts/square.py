#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
# Import 3D Plotting Library
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Import Kalman Filter Library
from pykalman import KalmanFilter
import numpy as np
import math

def apply_kalman_filter(x_values, y_values, z_values):
    # Define the initial state (position and velocity).
    initial_state = [x_values[0], y_values[0], z_values[0], 0, 0, 0]

    # Define the initial state covariance matrix.
    initial_state_covariance = np.eye(6)

    # Define the state transition matrix.
    transition_matrix = np.eye(6)
    transition_matrix[0, 3] = 1
    transition_matrix[1, 4] = 1
    transition_matrix[2, 5] = 1

    # Define the observation matrix.
    observation_matrix = np.zeros((3, 6))
    observation_matrix[0, 0] = 1
    observation_matrix[1, 1] = 1
    observation_matrix[2, 2] = 1

    # Create the Kalman Filter.
    kf = KalmanFilter(
        initial_state_mean=initial_state,
        initial_state_covariance=initial_state_covariance,
        transition_matrices=transition_matrix,
        observation_matrices=observation_matrix,
    )

    # Apply the Kalman Filter to the position data.
    measurements = np.column_stack((x_values, y_values, z_values))
    filtered_state_means, filtered_state_covariances = kf.filter(measurements)

    return filtered_state_means[:, 0], filtered_state_means[:, 1], filtered_state_means[:, 2]


def plot_trajectory(x_values, y_values, z_values, x_est, y_est, z_est):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(x_values, y_values, z_values, label="Drone Trajectory (Actual)", color="blue", marker="o")
    ax.plot(x_est, y_est, z_est, label="Drone Trajectory (Estimated)", color="green", linestyle="--", marker="x")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.legend()
    plt.show()

def sliding_mode_control(current_state, desired_state, params):
    x, y, z, dx, dy, dz = current_state
    x_d, y_d, z_d, dx_d, dy_d, dz_d = desired_state
    k, epsilon = params

    # Calculate the position and velocity errors
    e_pos = np.array([x_d - x, y_d - y, z_d - z])
    e_vel = np.array([dx_d - dx, dy_d - dy, dz_d - dz])

    # Calculate the sliding surface
    s = e_vel + k * e_pos

    # Calculate the control input
    u = -k * e_vel - (k ** 2) * e_pos - (epsilon * np.sign(s))

    return u

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Specify some waypoints
    goals = [
             [0, 0, 3, 0], 
             [5, 0, 3, -90], 
             [5, 5, 3, 0],
             [0, 5, 3, 90], 
             [0, 0, 3, 180], 
             [0, 0, 3, 0]
            ]
    i = 0

    x_values = []
    y_values = []
    z_values = []

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            # Get the current state of the drone
            current_position = drone.get_current_position() if hasattr(drone, 'get_current_position') else None
            current_velocity = drone.get_current_velocity() if hasattr(drone, 'get_current_velocity') else None
            if current_position is not None and current_velocity is not None:
                x, y, z = current_position.x, current_position.y, current_position.z
                dx, dy, dz = current_velocity.x, current_velocity.y, current_velocity.z

                # Calculate the control input using Sliding Mode Control
                current_state = [x, y, z, dx, dy, dz]
                desired_state = [goals[i][0], goals[i][1], goals[i][2], 0, 0, 0]  # Assuming desired velocities are zero
                control_params = (1, 0.5)  # Define your control parameters here (k, epsilon)
                control_input = sliding_mode_control(current_state, desired_state, control_params)

            # Use the control input to update the destination
            drone.set_destination(
                x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3], vx=control_input[0], vy=control_input[1], vz=control_input[2]
            )
            rate.sleep()
            if drone.check_waypoint_reached():
                x_values.append(goals[i][0])
                y_values.append(goals[i][1])
                z_values.append(goals[i][2])
                i += 1
                
    # Land after all waypoints is reached.
    x_est, y_est, z_est = apply_kalman_filter(x_values, y_values, z_values)
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    plot_trajectory(x_values, y_values, z_values, x_est, y_est, z_est)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
