import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

matplotlib.use('Agg')

latitude_history = []
longitude_history = []
altitude_history = []

def save(location):
    global latitude_history
    global longitude_history
    global altitude_history
    latitude_history.append(location.lat)
    longitude_history.append(location.lon)
    altitude_history.append(location.alt)
    
def show(filename="3d_plot.png"):
    global latitude_history
    global longitude_history
    global altitude_history
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_label("Latitude")
    ax.set_ylabel("Longitude")
    ax.set_zlabel("Altitude")
    ax.plot(latitude_history, longitude_history, altitude_history)
    
    plt.savefig(filename)
    plt.show()