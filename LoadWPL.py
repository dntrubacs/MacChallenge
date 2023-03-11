"""
Created by Daniel-Iosif Trubacs for the MAC team on 8 March 2023. The purpose of this module is to load an
array of Waypoints from a .txt and arrange them in different tracks. A track should be a simple numpy array
containing the waypoints (in order) that should be followed). The data in .txt file should be in the NMEA format.
"""

import numpy as np


def load_wpl(txt_file: str) -> list:
    """ Loads a series of waypoint from a .txt file and return the track expected to be followed.

    Args:
        txt_file: path to data file (waypoints must be in the NMEA format). An example of
                  a waypoint is: $MMWPL,5050.710799,N,00044.755897,W,WPT 1. The waypoints will
                  be then arranged in different tracks according to the given commands. An example of
                  of a set track command is: '$MMRTE,2,2,c,TRACK 1,WPT 6,WPT 7,WPT 8'
    Returns:
        (n_tracks, n_waypoints, lat, lon): list containing the list of waypoints values arranged in tracks as numpy arrays
    """
    # read the .txt file
    f = None
    try:
        f = open(txt_file, 'r')
    except FileNotFoundError as err:
        print('File name incorrect', err)

    # read the data and output a set of numpy array containing the waypoints
    if f is not None:
        # split into lines
        lines = f.readlines()

        # the array containing the waypoints
        waypoints = []

        # array containing the names of waypoints (e.g 'WPT 1')
        #  the name of waypoints[i] is given by waypoints_name[i]
        waypoints_name = []

        # load the waypoints
        for line in lines:
            # check whether the command starts with '$WMPL'
            line_split = line.split(',')
            if line_split[0] == '$MMWPL':
                # append the waypoint data (latitude, longitude, waypoint_index)
                waypoints.append(np.array([float(line_split[1]), float(line_split[3])]))
                waypoints_name.append(line_split[5].strip())

        # the array containing all the tracks
        tracks = []

        # go over all the tracks and extract the required waypoints
        # load the waypoints
        for line in lines:
            # check whether the command starts with '$MMRTE'
            line_split = line.split(',')
            if line_split[0] == '$MMRTE':
                # the current waypoints that make up the current track
                track_wp_name = [line_split[i].strip() for i in range(len(line_split)) if line_split[i].strip()
                                 in waypoints_name]

                # the current track containing the waypoints
                current_track = np.array([waypoints[waypoints_name.index(x)] for x in track_wp_name])

                # add the current track to the list of tracks
                tracks.append(current_track)

        # return the numpy array containing the tracks
        return tracks


if __name__ == '__main__':
    tracks_test = load_wpl('data.txt')
    print(tracks_test[0][0])
