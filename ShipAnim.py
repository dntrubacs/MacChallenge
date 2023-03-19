""" Created by Daniel-Iosif Trubacs and Flavio Gheri for the MAChellenge on 16 March 2023. The main aim of
this module is to create visualization for the ship movement.
"""

import numpy as np
from matplotlib import pyplot as plt


def find_limits(initial_position: np.ndarray, waypoints: np.ndarray) -> np.ndarray:
    """Creates limits for the plots

    Args:
        initial_position: the initial position of the boat (lat, lon) in DEG format
        waypoints: list of all waypoints in DEG format

    Returns: [[lon_min, lon_max], [lat_min,lat_max]]

    """
    # the range for lat and long for the waypoints and add 10% to it
    range_list = np.concatenate((waypoints, np.expand_dims(initial_position, axis=0)))

    lat_range = 3 * (np.max(range_list[:, 0]) - np.min(range_list[:, 0]))
    lon_range = 3 * (np.max(range_list[:, 1]) - np.min(range_list[:, 1]))

    # lat limits for the plot and add 15%
    lat_limit = np.array([np.max(range_list[:, 0]) - lat_range / 2, np.min(range_list[:, 0]) + lat_range / 2])
    lon_limit = np.array([np.max(range_list[:, 1]) - lon_range / 2, np.min(range_list[:, 1]) + lon_range / 2])

    return np.array([lon_limit, lat_limit])


def set_plot(waypoints: np.ndarray, current_pos: np.ndarray, current_speed: float,
             current_err: float, limits: np.ndarray, current_heading: float,
             path: np.ndarray, axis):
    """ Draws the waypoints on interactive plot. Latitude is plotted on the y axis and longitude is plotted on
        the x axis. All lat and lon values should be in DEG format.

    Args:
        waypoints: numpy array representing list of waypoints (n_waypoints, lat ,long)
        current_pos: current position of the bloat (lat, long)
        current_speed: current speed of the bloat
        current_err: current cross track error in relation to the current and last waypoint
        limits: lat and long limits of the current plot [[lat_min,lat_max], [long_min, long_max]]
        current_heading: current heading of the boat
        path: the path followed by the boat should, ([past_lat_pos, past_lon_positions])
        axis: matplotlib object representing the axis on which the data is plotted (axis.plot)

    """
    # waypoint range used to for correct location of plotting
    waypoint_range = [limits[0][1] - limits[0][0], limits[1][1] - limits[1][0]]

    # set the label of the plot
    axis.set_xlabel('Longitude')
    axis.set_ylabel('Latitude')

    # set the limits of the plot
    plt.xlim(limits[0])
    plt.ylim(limits[1])

    # plotting the waypoints
    for i in range(len(waypoints)):
        axis.plot(waypoints[i][1], waypoints[i][0], marker='x', color='red',  markersize=10)
        axis.text(waypoints[i][1], waypoints[i][0], 'WP' + str(i + 1))
        
    # plot a dashed line between the waypoints
    axis.plot(waypoints[:, 1], waypoints[:, 0], color='red', lw=1, linestyle='dashed' ) 
    
    # plotting the current position
    axis.plot(current_pos[1], current_pos[0], color='blue', markersize=5, marker='o')
    plt.title('Position: ' + str(round(current_pos[0], 6)) + '$^o$ '
              + str(round(current_pos[1], 6)) + '$^o$')

    # showing the speed
    axis.text(waypoints[0, 1] + waypoint_range[1] * 0.4, waypoints[0, 0] + waypoint_range[0] * 0.25,
              'Speed: ' + str(round(current_speed, 3))+ ' kts')

    # showing the current track error (in degrees)
    axis.text(waypoints[0, 1] + waypoint_range[1] * 0.4, waypoints[0, 0] + waypoint_range[0] * 0.275,
              'CT error: ' + str(round(current_err, 3)) + ' m')

    # showing the heading (in degrees)
    axis.text(waypoints[0, 1] - waypoint_range[1] * 1.2, waypoints[0, 0] + waypoint_range[0] * 0.275,
              'Heading:' + str(round(current_heading, 3)) + '$^o$')

    # draw an arrow pointing to the heading
    plt.annotate("", xy=(current_pos[1] + 0.0001*np.cos(current_heading), current_pos[0] + 0.0001*np.sin(current_heading)),
                 xytext=(current_pos[1], current_pos[0]), arrowprops=dict(arrowstyle="->"))

    # show the path followed by the boat
    axis.plot(path[1], path[0], lw=2, markersize=10, color='black')

    # invert x axis as longitude in the west
    plt.gca().invert_xaxis()



