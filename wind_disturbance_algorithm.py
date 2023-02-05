import time
import numpy as np
import pandas

kp = 0.75
ki = 1.5
kd = 0.0

k = 2
integral1 = [0, 0]
derivative1 = [0, 0]
output = [0, 0]

wind = [0, 0, 10]

start_location = [-7.276473, 112.793746, 2]
target_location = [-7.276766, 112.793957, 2]
current_location =  [0.0, 0.0]
error = [0, 0, 0]
previous_error = [0, 0, 0]

iteration = 10
speed = 5 # m/s
count = 0
integral_error = [0, 0]
derivative = [0, 0]
sliding_var = 0

current_location = start_location

while True:
  error = [target_location[0] - start_location[0] + wind[0],
           target_location[1] - start_location[1] + wind[1]]
  
  # integral1[0] = float(integral1[0] + error[0]) * float(0.001)
  # derivative1[0] = float(error[0] - previous_error[0]) / float(0.001)
  # output[0] = float(kp * error[0] + ki * integral1[0] + kd * derivative1[0])
  # print("integral1_x: ", integral1[0])
  # print("derivative1_x: ", derivative1[0])
  # print("output_x: ", output[0])
  
  # integral1[1] = float(integral1[1] + error[1]) * float(0.001)
  # derivative1[1] = float(error[1] - previous_error[1]) / float(0.001)
  # output[1] = float(kp * error[1] + ki * integral1[1] + kd * derivative1[1])
  # print("integral1_y: ", integral1[1])
  # print("derivative1_y: ", derivative1[1])
  # print("output_y: ", output[1])
  
  # current_location[0] += output[0]
  # current_location[1] += output[1]
  # print("location now (x): ", output[0])
  # print("location now (y): ", output[1])
  
  s = (lambda x:  error + k / x * np.sign(error))(k)
  u = -k * np.sign(s)
  print("s: ", s)
  print("u: ", u)
  print("s[0]: ", u.item(0))
  print("s[1]: ", u.item(1))
  
  # print("ki x error[1]", (ki * integral_error[1]))
  # print("kd x error[1]", (kd * derivative[1]))
  
  print("error", error)
  integral_error[0] += error[0]
  integral_error[1] += error[1]
  # print("integral error", integral_error)
  derivative = [error[0] - previous_error[0],
                error[1] - previous_error[1]]
  # print("derivative error", derivative)
  
  # desired_position = [kp * error[0] + kd * (error[0] - previous_error[0]),
  #                     kp * error[1] + kd * (error[1] - previous_error[1])]
  u_x = kp * error[0] + ki * integral_error[0] + kd * derivative[0]
  u_y = kp * error[1] + ki * integral_error[1] + kd * derivative[1]

  desired_position = [u_x, u_y]
  # print("Desired Position: ", count)
  # print("X: ", desired_position[0])
  # print("Y: ", desired_position[1])
  # print("Throttle Values: ", count)
  # print("1: ", (int(1500 + desired_position[0])))
  # previous_error = error
  # print("2: ", (int(1500 + desired_position[1])))
  count += 1
  if count >= iteration:
    break
  
  # 1500
  