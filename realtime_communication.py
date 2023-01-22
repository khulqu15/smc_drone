import dronekit
import time

vehicle = dronekit.connect('/dev/ttyACM0', baud=115200, wait_ready=True)

@vehicle.on_message('*')
def handle_message(self, name, message):
    print(name, message)

while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    print("Velocity: ", vehicle.velocity)
    print("Attitude: ", vehicle.attitude)

    time.sleep(1)

vehicle.close()
