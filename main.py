import dronekit
import smckf
import smckf_autonomous
import smc
import smc_autonomous

print("[0] Simulation")
print("[1] Testing")
option = int(input("Choose one: ") or 1)
print("[0] 57600")
print("[1] 115200")
baudrate = int(input("Choose baudrate: ") or 0)
print("[0] Sliding Mode Control")
print("[1] Kalman Filter & Sliding Mode Control")
print("[2] Sliding Mode Control (Auto)")
print("[3] Kalman Filter & Sliding Mode Control (Auto)")
method = int(input("Choose one: ") or 3)
print("[0] Default")
print("[1] Red Field")
print("[2] Basket Field")
environment = int(input("Choose environment: ") or 0)
connections = [
    "udp:127.0.0.1:14551",
    "/dev/ttyUSB0"
]

baudrates = [
    57600,
    115200,
]

takeoff_alt = float(input("Takeoff Altitude (m): ") or 1)
trajectory_alt = float(input("Trajectory Altitude (m): ") or 1)
trajectory_distance = float(input("Trajectory Distance (m): ") or 10)
trajectory_duration = float(input("Trajectory Duration Fly (s): ") or 5)
override_speed = float(input("Override Speed (s): ") or 10)

is_plotting = input("Plotting Diagram ? [Y/N] : ") or "Y"
is_scanning = input("Scanning distance altitude with Lidar ? [Y/N] : ") or "Y"
wait_for_ready = input("Wait Vehicle Ready ? [Y/N] : ") or "Y"

if is_plotting.lower() == 'y': plotting = True
else: plotting = False

if is_scanning.lower() == 'y': scanning = True
else: scanning = True

if wait_for_ready.lower() == 'y': waiting = True
else: waiting = False

print("Option : ", [connections[option], type(connections[option])], )
print("Baudrate : ", [baudrates[baudrate], type(baudrates[baudrate])])
print("Is Waiting : ", [waiting, type(waiting)])
vehicle = dronekit.connect(connections[option], baud=baudrates[baudrate], wait_ready=waiting, timeout=60)


if method == 0:
    smc.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smc.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    smc.landing_disarm(vehicle, scanning, plotting)
elif method == 1 :
    smckf.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smckf.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    smckf.landing_disarm(vehicle, scanning, plotting)
elif method == 2:
    smc_autonomous.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smc_autonomous.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting, environment)
    smc_autonomous.landing_disarm(vehicle, scanning, plotting)
elif method == 3:
    smckf_autonomous.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smckf_autonomous.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting, environment)
    smckf_autonomous.landing_disarm(vehicle, scanning, plotting)
    
vehicle.close()