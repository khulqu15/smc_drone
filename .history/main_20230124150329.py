import dronekit
import time
import pykalman

altitute_target = 1
vehicle = dronekit.connect('/dev/ttyACM0', wait_ready=True)
vehicle.armed = True
vehicle.simple_takeoff(altitute_target)

def sliding_mode_control(vehicle, desired_altitude):
    current_altitude = vehicle.location.global_relative_frame.alt
    error = desired_altitude - current_altitude
    if abs(error) < 0.1:
        vehicle.channels.overrides = {'3': 1500}
    elif error > 0:
        vehicle.channels.overrides = {'3': 1600}
    else:
        vehicle.channels.overrides = {'3': 1400}
  
while True:
  sliding_mode_control(vehicle, altitute_target)
  attitude = vehicle.attitude
  print("Roll: %f, Pitch: %f, Yaw: %f, Alt: %f" % (attitude.roll, attitude.pitch, attitude.yaw, vehicle.location.global_relative_frame.alt))
  if vehicle.location.global_relative_frame.alt >= 1.2*0.95:
      print("Reached altitude")
      break
  time.sleep(1)

time.sleep(30)

# Land
vehicle.mode = dronekit.VehicleMode("LAND")

vehicle.close()