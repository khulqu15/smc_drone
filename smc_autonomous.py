import dronekit
import time
import numpy as np
import plot
import random

current_location = dronekit.LocationGlobalRelative(0,0,0)

def get_position_estimation(location):
    latitude = location.lat + (random.randrange(-2, 2) / 10000000)
    longitude = location.lon + (random.randrange(-2, 2) / 10000000)
    altitude = location.alt + (random.randrange(-2, 2) / 10000000)
    estimation = dronekit.LocationGlobalRelative(latitude, longitude, altitude)
    print(estimation)
    return estimation


def arm_takeoff(vehicle, altitude, scanning, plotting):
    global current_location
    current_location = vehicle.location.global_relative_frame
    print("Arming motors")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    print("Take off!")
    vehicle.simple_takeoff(altitude)
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        print("Take Off Data : ")
        print(vehicle.location.global_relative_frame)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, vehicle.attitude, estimation_position, "SMC_AUTO", altitude)
        print("---------------------------------------------------------------------------")

        
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print('Reached target altitude: %s' % altitude)
            break
        
        time.sleep(0.1)


def trajectory(vehicle, altitude, distance, duration, speed, scanning, plotting, environment):
    global current_location
    vehicle.groundspeed = 15
    current_location = vehicle.location.global_relative_frame
    print("Trajectory")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    desired_location1 = dronekit.LocationGlobalRelative(-7.770292, 112.753611, altitude)
    vehicle.simple_goto(desired_location1)
    start_time = time.time()
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        drone_location = vehicle.location.global_relative_frame
        target_distance = desired_location1.lat - drone_location.lat
        print("Trajectory : ")
        print("Target Distance: ", target_distance)
        print(vehicle.location.global_relative_frame)
        print("Attitude : ", vehicle.attitude)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, vehicle.attitude, estimation_position, "SMC_AUTO", altitude)
        print("---------------------------------------------------------------------------")
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            break
        time.sleep(1)
                  
def landing_disarm(vehicle, scanning, plotting):
    print("Landing")
    vehicle.mode = dronekit.VehicleMode("LAND")
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        print("Landing : ")
        print(vehicle.location.global_relative_frame)
        print("Attitude : ", vehicle.attitude)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, vehicle.attitude, estimation_position, "SMC_AUTO", vehicle.location.global_relative_frame.alt)
        print("---------------------------------------------------------------------------")
        
        if vehicle.location.global_relative_frame.alt <= 0.2:
            break
    
    print('Disarming...')
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)
        
    if plotting: plot.show("smckfauto.png")
    
    vehicle.close()
    exit()


    
