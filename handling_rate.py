import time
import dronekit

vehicle = dronekit.connect('/dev/ttyACM0', baud=115200, wait_ready=True)

loop_rate = 10

start_time = time.time()

while True:
    # Compute the control input
    # ...

    # Set the attitude target
    vehicle.attitude_target_local_ned = control_input

    elapsed_time = time.time() - start_time
    time_step = 1.0 / loop_rate
    time.sleep(time_step - elapsed_time)

    start_time = time.time()

vehicle.close()