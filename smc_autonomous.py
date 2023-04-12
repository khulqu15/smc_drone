import dronekit
import time
import numpy as np
import plot
import random

current_location = dronekit.LocationGlobalRelative(0,0,0)

def get_position_estimation(location):
    latitude = location.lat + (random.randrange(-15, 15) / 10000000)
    longitude = location.lon + (random.randrange(-15, 15) / 10000000)
    altitude = location.alt + (random.randrange(-15, 15) / 10000000)
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
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", altitude)
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
    if environment == 0:
        desired_location1 = dronekit.LocationGlobalRelative(current_location.lat, current_location.lon, altitude)
        desired_location2 = dronekit.LocationGlobalRelative(current_location.lat + distance/1000000, current_location.lon, altitude)
        desired_location3 = dronekit.LocationGlobalRelative(current_location.lat, current_location.lon + distance/1000000, altitude)
        desired_location4 = dronekit.LocationGlobalRelative(current_location.lat - distance/1000000, current_location.lon, altitude)
        desired_location5 = dronekit.LocationGlobalRelative(current_location.lat, current_location.lon - distance/1000000, altitude)
    
    if environment == 1:
        desired_location1 = dronekit.LocationGlobalRelative(-7.276724, 112.794938, altitude)
        desired_location2 = dronekit.LocationGlobalRelative(-7.276724, 112.794960, altitude)
        desired_location3 = dronekit.LocationGlobalRelative(-7.276742, 112.794958, altitude)
        desired_location4 = dronekit.LocationGlobalRelative(-7.276746, 112.794940, altitude)
        desired_location5 = dronekit.LocationGlobalRelative(-7.276724, 112.794938, altitude)
    
    if environment == 2:    
        desired_location1 = dronekit.LocationGlobalRelative(-7.276613, 112.793801, altitude)
        desired_location2 = dronekit.LocationGlobalRelative(-7.276617, 112.793834, altitude)
        desired_location3 = dronekit.LocationGlobalRelative(-7.276645, 112.793832, altitude)
        desired_location4 = dronekit.LocationGlobalRelative(-7.276645, 112.793801, altitude)
        desired_location5 = dronekit.LocationGlobalRelative(-7.276613, 112.793801, altitude)
    
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
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", altitude)
        print("---------------------------------------------------------------------------")
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            break
        time.sleep(1)
        
    start_time = time.time()
    vehicle.simple_goto(desired_location2)
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        drone_location = vehicle.location.global_relative_frame
        target_distance = desired_location2.lon - drone_location.lon
        print("Trajectory : ")
        print("Target Distance: ", target_distance)
        print(vehicle.location.global_relative_frame)
        print("Attitude : ", vehicle.attitude)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", altitude)
        print("---------------------------------------------------------------------------")
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            break
        time.sleep(1)
    
    start_time = time.time()
    vehicle.simple_goto(desired_location3)
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        drone_location = vehicle.location.global_relative_frame
        target_distance = desired_location3.lat - drone_location.lat
        print("Trajectory : ")
        print("Target Distance: ", target_distance)
        print(vehicle.location.global_relative_frame)
        print("Attitude : ", vehicle.attitude)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", altitude)
        print("---------------------------------------------------------------------------")
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            break
        time.sleep(1)
    
    start_time = time.time()
    vehicle.simple_goto(desired_location4)
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        drone_location = vehicle.location.global_relative_frame
        target_distance = desired_location4.lon - drone_location.lon
        print("Trajectory : ")
        print("Target Distance: ", target_distance)
        print(vehicle.location.global_relative_frame)
        print("Attitude : ", vehicle.attitude)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", altitude)
        print("---------------------------------------------------------------------------")
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            break
        time.sleep(1)
        
        
    start_time = time.time() 
    vehicle.simple_goto(desired_location5)
    while True:
        estimation_position = get_position_estimation(vehicle.location.global_relative_frame)
        drone_location = vehicle.location.global_relative_frame
        target_distance = desired_location5.lon - drone_location.lon
        print("Trajectory : ")
        print("Target Distance: ", target_distance)
        print(vehicle.location.global_relative_frame)
        print("Attitude : ", vehicle.attitude)
        if scanning: print("Lidar Sensor Distance (m)", vehicle.location.global_relative_frame.alt)
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", altitude)
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
        if plotting: plot.save(vehicle.location.global_relative_frame, estimation_position, "SMC_AUTO", vehicle.location.global_relative_frame.alt)
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


    
