import matplotlib.pyplot as plt
import numpy as np
from ShipAnim import set_plot, find_limits
from LoadWPL import load_wpl
from LOS_guidance import DMM_to_DEG
import time

# load the waypoints from data.txt file
tracks = load_wpl('data.txt')
waypoints_list = []
j = 0
for track in tracks:
    for waypoint_dmm in track:
        j += 1
        waypoints_list.append([DMM_to_DEG(waypoint_dmm)[0], DMM_to_DEG(waypoint_dmm)[1]])
        print('waypoint', j, 'dmm', waypoint_dmm, 'deg', DMM_to_DEG(waypoint_dmm))

# change to numpy array
waypoints_list = np.array(waypoints_list)

# Create an empty list to store the data
lat_data = np.arange(50.845140, 50.84518, 0.0000001)
lon_data = np.arange(0.745076, 0.745930, 0.000001)


# create random data for speed, error and heading
theta = np.arange(1, 31.4, 0.1)
speed = np.arange(1, 31.4, 0.1)
error = np.arange(1, 31.4, 0.1)

# initial position
initial_pos = np.array(DMM_to_DEG(np.array([5050.708799, 44.755897])))


# the limits for the plot
pl_limits = find_limits(initial_position=initial_pos, waypoints=waypoints_list)

# Initialize the time and frequency variables
t = 0
freq = 1

# Define a function to handle the keyboard interrupt event
def on_key_press(event):
    if event.key == 'p':
        fig.canvas.stop_event_loop()

# Create an empty plot with axis labels
fig, ax = plt.subplots()
plt.ion()

# Connect the keyboard interrupt handler to the plot window
fig.canvas.mpl_connect('key_press_event', on_key_press)
""" Created by Daniel-Iosif Trubacs and Flavio Gheri for the MAChellenge on 16 March 2023. The main aim of
this module is to create visualization for the ship movement.
"""

# Continuously generate new data and update the plot
past_lat = []
past_lon = []

while True:
    #plt.annotate("", xy=(data_x[t]+np.cos(theta[t]), data_y[t]+np.sin(theta[t])), xytext=(data_x[t], data_y[t]),
                #arrowprops=dict(arrowstyle="->"))

    if t == 0:
     set_plot(waypoints=waypoints_list, current_pos = np.array([lat_data[0], lon_data[-1]]), current_speed=speed[t],
             current_err=error[t], limits=pl_limits, current_heading=theta[t], path=track, axis=ax)
     past_lat.append(lat_data[0])
     past_lon.append(lon_data[-1])
    else:
     set_plot(waypoints=waypoints_list, current_pos=np.array([lat_data[t], lon_data[-t]]), current_speed=speed[t],
                 current_err=error[t], limits=pl_limits, current_heading=theta[t], path=track, axis=ax)
     past_lat.append(lat_data[t])
     past_lon.append(lon_data[-t])
    track = np.array([past_lat, past_lon])



    plt.draw()
    plt.pause(0.01)

    # Increment the time variable
    t += 1

    # Check if a keyboard interrupt event has occurred
    if not plt.fignum_exists(fig.number):
        break

    # Clear the plot to allow for a live update
    ax.cla()


# Turn off interactive plotting
plt.ioff()

# Show the final plot
plt.show()
