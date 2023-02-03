import time

kp = 0.5
ki = 0.1
kd = 0.2

wind = [10, 9]

start_location = [-35.363262, 149.165374]
target_location = [-35.363233, 149.165396]

error = [0, 0]
previous_error = [0, 0]

iteration = 100
count = 0
integral_error = [0, 0]
derivative = [0, 0]

while True:
  error = [target_location[0] - start_location[0] + wind[0],
           target_location[1] - start_location[0] + wind[1]]
  integral_error[0] += error[0]
  integral_error[1] += error[1]
  derivative = [error[0] - previous_error[0],
        error[1] - previous_error[1]]

  # desired_position = [kp * error[0] + kd * (error[0] - previous_error[0]),
  #                     kp * error[1] + kd * (error[1] - previous_error[1])]
  u_x = kp * error[0] + ki * integral_error[0] + kd * derivative[0]
  u_y = kp * error[1] + ki * integral_error[1] + kd * derivative[1]
  desired_position = [u_x, u_y]
  print("Desired Position: ", count)
  print("X: ", desired_position[0])
  print("Y: ", desired_position[1])
  print("Throttle Values: ", count)
  print("1: ", (int(1500 + desired_position[0])))
  previous_error = error
  print("2: ", (int(1500 + desired_position[1])))
  count += 1
  if count >= iteration:
    break
  