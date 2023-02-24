import dronekit
import smckf
import smc

print("[0] Simulation")
print("[1] Testing")
option = int(input("Choose one: "))
print("[0] 57600")
print("[1] 115200")
baudrate = int(input("Choose baudrate: "))
print("[0] Sliding Mode Control")
print("[1] Kalman Filter & Sliding Mode Control")
method = int(input("Choose one: "))
connections = [
    "udp:127.0.0.1:14551",
    "/dev/ttyUSB0"
]

baudrates = [
    57600,
    115200,
]

takeoff_alt = float(input("Takeoff Altitude (m): ") or 1.0)
trajectory_alt = float(input("Trajectory Altitude (m): ") or 1.5)
trajectory_distance = float(input("Trajectory Distance (m): ") or 2.0)
trajectory_duration = float(input("Trajectory Duration Fly (s): ") or 10.0)
override_speed = float(input("Override Speed (s): ") or 10.0)

is_plotting = input("Plotting Diagram ? [Y/N] : ") or "Y"
is_scanning = input("Scanning distance altitude with Lidar ? [Y/N] : ") or "Y"
wait_for_ready = input("Wait Vehicle Ready ? [Y/N] : ") or "Y"

if is_plotting.lower() == 'y': plotting = True
else: plotting = False

if is_scanning.lower() == 'y': scanning = True
else: scanning = True

if wait_for_ready.lower() == 'y': waiting = True
else: waiting = False

try: 
    vehicle = dronekit.connect(connections[int(option)], baud=baudrates[int(baudrate)], wait_ready=waiting, timeout=60)
except Exception as e:
    print("Failed to connect vehicle option : ", str(e))
    print("Trying to connect using ttyUSB0...")
    if option == 0: other_option = 1
    else: other_option = 0
    
    try:
        vehicle = dronekit.connect(connections[int(other_option)], wait_ready=True, timeout=60)
        print("Connection successful!")
    except Exception as e:
        print("Connection failed: " + str(e))
        exit(1)

if method == 0:
    smc.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smc.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    smc.landing_disarm(vehicle, scanning, plotting)
else :
    smckf.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smckf.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    smckf.landing_disarm(vehicle, scanning, plotting)

vehicle.close()