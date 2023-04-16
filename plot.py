import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from datetime import datetime
import random
# matplotlib.use('Agg')

latitude_history = []
longitude_history = []
altitude_history = []
est_latitude_history = []
est_longitude_history = []
est_altitude_history = []

pitch_history = []
roll_history = []
yaw_history = []
est_pitch_history = []
est_roll_history = []
est_yaw_history = []

output_csv = None
altitude_status = 0
category_status = None
def save(location, attitude, estimation, category, altitude):
    global latitude_history
    global longitude_history
    global altitude_history
    global est_latitude_history
    global est_longitude_history
    global est_altitude_history
    global altitude_status
    global category_status
    global pitch_history
    global roll_history
    global yaw_history
    global est_pitch_history
    global est_roll_history
    global est_yaw_history
    altitude_status = altitude
    category_status = category
    latitude_history.append(location.lat)
    longitude_history.append(location.lon)
    altitude_history.append(location.alt)
    est_latitude_history.append(estimation.lat)
    est_longitude_history.append(estimation.lon)
    est_altitude_history.append(estimation.alt)
    pitch_history.append(attitude.pitch / 10)
    roll_history.append(attitude.roll / 10)
    yaw_history.append(attitude.yaw / 10)
    est_pitch_history.append((attitude.pitch + random.randrange(-1, 2)) / 10)
    est_roll_history.append((attitude.roll + random.randrange(-1, 2)) / 10)
    est_yaw_history.append((attitude.yaw + random.randrange(-1, 2)) / 10)
    
def show(filename="3d_plot.png", is_show=False):
    global latitude_history
    global longitude_history
    global altitude_history
    global est_latitude_history
    global est_longitude_history
    global est_altitude_history
    global altitude_status
    global category_status
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_label("Latitude")
    ax.set_ylabel("Longitude")
    ax.set_zlabel("Altitude")
    ax.plot(latitude_history, longitude_history, altitude_history, color='b')
    ax.text(latitude_history[0], longitude_history[0], altitude_history[0], 'Actual Position', color='b')
    ax.plot(est_latitude_history, est_longitude_history, est_altitude_history, color='r')
    ax.text(est_latitude_history[0], est_longitude_history[0], est_altitude_history[0], 'Estimate Position', color='r')
    
    df = pd.DataFrame({
        'Latitude': latitude_history,
        'Longitude': longitude_history,
        'Altitude': altitude_history,
        'Est. Latitude': est_latitude_history,
        'Est. Longitude': est_longitude_history,
        'Est. Altitude': est_altitude_history,
        'Roll': roll_history,
        'Pitch': pitch_history,
        'Yaw': yaw_history
    })
    
    now = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    output_csv = now+"_"+category_status+"_"+str(altitude_status)+".xlsx"
    
    new_filename = now+"_"+category_status+"_"+str(altitude_status)+"_"+filename
    
    writer = pd.ExcelWriter(output_csv, engine='xlsxwriter')
    df.to_excel(writer, sheet_name='Sheet1', index=False)
    writer.save()
    
    csv = df.to_csv(index=False)
    with open('data.csv', 'w') as f:
        f.write(csv)

    plt.savefig(new_filename)
    if is_show:
        plt.show()