import dronekit
import time

vehicle = dronekit.connect('/dev/ttyACM0', baud=115200, wait_ready=True)

vehicle.wait_ready(timeout=120)

min_distance = 0.5 # meters

while True:
    scan_data = vehicle.rangefinder.distance

    if scan_data < min_distance:
        print("Obstacle detected at distance: ", scan_data)
    else:
        print("No obstacle detected.")

    time.sleep(0.1)

vehicle.close()