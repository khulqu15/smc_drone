import dronekit
import smckf
import smc
import autonomous
import pymavlink
import time

print("[0] Simulation")
print("[1] Testing")
option = int(input("Choose one: ") or 0)
print("[0] 57600")
print("[1] 115200")
baudrate = int(input("Choose baudrate: ") or 1)
print("[0] Sliding Mode Control")
print("[1] Kalman Filter & Sliding Mode Control")
print("[2] Kalman Filter & Sliding Mode Control (Auto)")
method = int(input("Choose one: ") or 2)
connections = [
    "udp:127.0.0.1:14551",
    "/dev/ttyUSB0"
]

baudrates = [
    57600,
    115200,
]

takeoff_alt = float(input("Takeoff Altitude (m): ") or 2)
trajectory_alt = float(input("Trajectory Altitude (m): ") or 3)
trajectory_distance = float(input("Trajectory Distance (m): ") or 30)
trajectory_duration = float(input("Trajectory Duration Fly (s): ") or 5)
override_speed = float(input("Override Speed (s): ") or 5)

is_plotting = input("Plotting Diagram ? [Y/N] : ") or "Y"
is_scanning = input("Scanning distance altitude with Lidar ? [Y/N] : ") or "Y"
wait_for_ready = input("Wait Vehicle Ready ? [Y/N] : ") or "Y"

if is_plotting.lower() == 'y': plotting = True
else: plotting = False

if is_scanning.lower() == 'y': scanning = True
else: scanning = True

if wait_for_ready.lower() == 'y': waiting = True
else: waiting = False

# try:
print("Option : ", [connections[option], type(connections[option])], )
print("Baudrate : ", [baudrates[baudrate], type(baudrates[baudrate])])
print("Is Waiting : ", [waiting, type(waiting)])
vehicle = dronekit.connect(connections[option], baud=baudrates[baudrate], wait_ready=waiting, timeout=60)
# except Exception as e:
#     print("Failed to connect vehicle option : ", str(e))
#     print("Trying to connect other options...")
#     if option == 0: other_option = 1
#     else: other_option = 0
    
#     try:
#         vehicle = dronekit.connect(connections[int(other_option)], wait_ready=True, timeout=60)
#         print("Connection successful!")
#     except Exception as e:
#         print("Connection failed: " + str(e))
#         exit(1)

# Set the drone's velocity in the x, y, and z directions
vx = 1  # m/s
vy = 1  # m/s
vz = 1  # m/s

# Create a SET_POSITION_TARGET_LOCAL_NED message
msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target system, target component
    pymavlink.mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
    0b0000111111000111,  # type_mask (only speeds enabled)
    0, 0, 0,             # x, y, z positions (not used)
    vx, vy, vz,          # x, y, z velocity in m/s
    0, 0, 0,             # x, y, z acceleration (not used)
    0, 0                 # yaw, yaw_rate (not used)
)

# Send the message
vehicle.send_mavlink(msg)

# Wait for 10 seconds
time.sleep(10)


if method == 0:
    smc.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smc.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    smc.landing_disarm(vehicle, scanning, plotting)
elif method == 1 :
    smckf.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    smckf.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    smckf.landing_disarm(vehicle, scanning, plotting)
else:
    autonomous.arm_takeoff(vehicle, takeoff_alt, scanning, plotting)
    autonomous.trajectory(vehicle, trajectory_alt, trajectory_distance, trajectory_duration, override_speed, scanning, plotting)
    # Stop the drone's movement
    vx = 0  # m/s
    vy = 0  # m/s
    vz = 0  # m/s

    # Create a SET_POSITION_TARGET_LOCAL_NED message to stop the drone's movement
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        pymavlink.mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000000,  # type_mask (only position enabled)
        0, 0, 0,             # x, y, z positions (not used)
        0, 0, 0,             # x, y, z velocity (not used)
        0, 0, 0,             # x, y, z acceleration (not used)
        0, 0                 # yaw, yaw_rate (not used)
    )
    autonomous.landing_disarm(vehicle, scanning, plotting)
    
vehicle.close()