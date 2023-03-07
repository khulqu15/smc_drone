import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# matplotlib.use('Agg')

latitude_history = []
longitude_history = []
altitude_history = []

est_latitude_history = []
est_longitude_history = []
est_altitude_history = []

def save(location, estimation):
    global latitude_history
    global longitude_history
    global altitude_history
    global est_latitude_history
    global est_longitude_history
    global est_altitude_history
    latitude_history.append(location.lat)
    longitude_history.append(location.lon)
    altitude_history.append(location.alt)
    est_latitude_history.append(estimation.lat)
    est_longitude_history.append(estimation.lon)
    est_altitude_history.append(estimation.alt)
    
def show(filename="3d_plot.png"):
    global latitude_history
    global longitude_history
    global altitude_history
    global est_latitude_history
    global est_longitude_history
    global est_altitude_history
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
        'Est. Altitude': est_altitude_history
    })
    
    writer = pd.ExcelWriter('data.xlsx', engine='xlsxwriter')
    df.to_excel(writer, sheet_name='Sheet1', index=False)
    writer.save()
    
    csv = df.to_csv(index=False)
    with open('data.csv', 'w') as f:
        f.write(csv)

    plt.savefig(filename)
    plt.show()