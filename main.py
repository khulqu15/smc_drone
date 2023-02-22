import dronekit
import smckf
import smc

print("[0] Simulation")
print("[1] Testing")
option = int(input("Choose one: "))
print("[0] Sliding Mode Control")
print("[1] Kalman Filter & Sliding Mode Control")
method = int(input("Choose one: "))
connections = [
    "udp:127.0.0.1:14551",
    "/dev/ttyACM0"
]

takeoff_alt = int(input("Takeoff Altitude (m): ") or 1)
trajectory_alt = int(input("Trajectory Altitude (m): ") or 1.5)
trajectory_distance = int(input("Trajectory Distance (m): ") or 2)
trajectory_duration = int(input("Trajectory Duration Fly (s): ") or 10)

is_plotting = input("Plotting Diagram ? [Y/N] : ") or "Y"
is_scanning = input("Scanning distance altitude with Lidar ? [Y/N]") or "Y"

if is_plotting.lower() == 'y': plotting = True
else: plotting = False

if is_scanning.lower() == 'y': scanning = True
else: scanning = True

vehicle = dronekit.connect(connections[int(option)], wait_ready=True, timeout=60)

if method == 0:
    smc.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smc.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, scanning, plotting)
    smc.landing_disarm(vehicle, scanning, plotting)
else :
    smckf.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smckf.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, scanning, plotting)
    smckf.landing_disarm(vehicle, scanning, plotting)

vehicle.close()