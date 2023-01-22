import dronekit

vehicle = dronekit.connect('/dev/ttyACM0', baud=115200, wait_ready=True)

def safety_check():
    if not vehicle.is_armable:
        print("Vehicle not armable, check for errors.")
        return False

    if vehicle.gps_0.fix_type < 2:
        print("No GPS fix, cannot arm.")
        return False

    if vehicle.mode.name != "GUIDED":
        print("Vehicle not in guided mode, switch to guided mode.")
        return False

    if vehicle.battery.level < 10:
        print("Battery level low, cannot arm.")
        return False

    return True

if safety_check():
    vehicle.armed = True
else:
    print("Safety check failed, cannot arm.")

vehicle.close()
