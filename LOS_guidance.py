"""
Created by Chihiro Hirai for the MAC team on 8 March 2023. The purpose of this module is to
calculate the desired heading from current postion, previous waypoint and next waypoint
"""

# Line Of Sight(LOS) guidance  based on GPS(latlon) cordinate
import math
import numpy as np
from numpy.linalg import norm
from math import sqrt, cos, sin, atan2, pi


# Convert the unit of Degree Minutes.Minutes to Degrees
def DMM_to_DEG(WP_DMM):
    lat = WP_DMM[0]
    lon = WP_DMM[1]

    # For latitude
    DMM_lat = str(lat).rjust(11, '0')
    D1_lat = round(float(str(DMM_lat[0] + DMM_lat[1])))  # Degrees, first two digits
    MM_lat = round(float(DMM_lat[2] + DMM_lat[3])) + round(math.modf(lat)[0], 6)  # Minutes
    D2_lat = round(MM_lat / 60, 6)  # Convert minute to degrees
    DEG_lat = D1_lat + D2_lat

    # For longitude
    DMM_lon = str(lon).rjust(12, '0')
    D1_lon = round(float(str(DMM_lon[0] + DMM_lon[1] + DMM_lon[2])))  # Degrees, first three digits
    MM_lon = round(float(DMM_lon[3] + DMM_lon[4])) + round(math.modf(lon)[0], 6)  # Minutes
    D2_lon = round(MM_lon / 60, 6)  # Convert minute to degrees
    DEG_lon = D1_lon + D2_lon
    return (DEG_lat, DEG_lon)

def call_distance(P1, P2):
        # Set radius of earth
        pole_radius = 6356752.314245  # Pole radius of earth
        equator_radius = 6378137.0  # Equator radius

        # Convert latlon to radians
        lat_P1 = math.radians(P1[0])
        lon_P1 = math.radians(P1[1])
        lat_P2 = math.radians(P2[0])
        lon_P2 = math.radians(P2[1])

        lat_difference = lat_P1 - lat_P2  # Difference of latitude
        lon_difference = lon_P1 - lon_P2  # Difference of longitude
        lat_average = (lat_P1 + lat_P2) / 2  # Mean latitude

        e2 = (math.pow(equator_radius, 2) - math.pow(pole_radius, 2)) \
             / math.pow(equator_radius, 2)  # major eccentricity^2

        w = math.sqrt(1 - e2 * math.pow(math.sin(lat_average), 2))

        m = equator_radius * (1 - e2) / math.pow(w, 3)  # meridian curvature radius

        n = equator_radius / w  # prime vertical curvature radius

        distance = math.sqrt(math.pow(m * lat_difference, 2) \
                             + math.pow(n * lon_difference * math.cos(lat_average), 2))  # Distance,m

        return distance, m * lat_difference, n * lon_difference


# TIP: you can access the position and waypoints with .x and .y accessors.
# Example: position.x
# Example: previous_waypoint.y
# Calc for distance on GPS cordinate
def latlon_meter_convertor(P):
        # Set radius of earth
        pole_radius = 6356752.314245  # Pole radius of earth
        equator_radius = 6378137.0  # Equator radius
        x = 360 / (equator_radius * math.cos(math.radians(P[0])) * 2 * np.pi)  # degree/m in X axis
        y = 360 / (pole_radius * 2 * np.pi)  # degree/m in Y axis
        return x, y

def LOS_latlon(position: np.ndarray,
               previous_waypoint: np.ndarray,
               current_waypoint: np.ndarray,
               los_radius: float = 15.0,
               debug=False) -> np.ndarray:
    """

    Args:
        position: Latitude and longitude in DMM format
        previous_waypoint: Latitude and longitude in DMM format
        current_waypoint: Latitude and longitude in DMM format
        los_radius: control parameter
        debug: whether all parameter should be printed

    Returns:
        heading and error

    """
    # changing from DMM to DEG format
    position = DMM_to_DEG(position)
    previous_waypoint = DMM_to_DEG(previous_waypoint)
    current_waypoint = DMM_to_DEG(current_waypoint)


    # ************************Main code for LOS******************************************************
    # Angle of path
    # alpha = atan2((current_waypoint.y-previous_waypoint.y),(current_waypoint.x-previous_waypoint.x))
    alpha = atan2(call_distance(current_waypoint, previous_waypoint)[1],
                  call_distance(current_waypoint, previous_waypoint)[2])
    # Along-track distance (los_s) and cross-track error (los_e)

    # los_s =((position.x-previous_waypoint.x)*cos(alpha)+(position.y-previous_waypoint.y)*sin(alpha))
    los_s = ((call_distance(position, previous_waypoint)[2]) * cos(alpha) + (
    call_distance(position, previous_waypoint)[1]) * sin(alpha))

    # second_angle=atan2(position.y-previous_waypoint.y,position.x-previous_waypoint.x) + alpha
    second_angle = atan2(call_distance(position, previous_waypoint)[1],
                         call_distance(position, previous_waypoint)[2]) + alpha

    # los_e = sqrt((position.x-previous_waypoint.x)**2+(position.y-previous_waypoint.y)**2)*sin(second_angle)
    los_e = sqrt((call_distance(position, previous_waypoint)[2]) ** 2 + (
    call_distance(position, previous_waypoint)[1]) ** 2) * sin(second_angle)

    los_delta = 0.0  # this is correct

    # Compute lookahead distance (los_delta). It is always positive
    if los_radius > abs(los_e):
        los_delta = sqrt(los_radius ** 2 - los_e ** 2)

    # Orthogonal projection (where the blue line turns red)*****working for this
    # xproj = (los_s )*cos(alpha)+previous_waypoint.x
    # yproj = (los_s )*sin(alpha)+previous_waypoint.y
    deg_meter_x = latlon_meter_convertor(position)[0]
    deg_meter_y = latlon_meter_convertor(position)[1]

    lon_proj = (los_s + los_delta) * cos(alpha) * deg_meter_x + previous_waypoint[
        1]  # lon projection in degrees + lon of previousWP
    lat_proj = (los_s + los_delta) * sin(alpha) * deg_meter_y + previous_waypoint[
        0]  # lat projection in degrees + lat of previous WP

    TargetP = (lat_proj, lon_proj)
    # Heading point
    # losx = xproj+(los_delta)*cos(alpha) - position.x
    los_lon = call_distance(TargetP, position)[2]

    # losy = yproj+(los_delta)*sin(alpha) - position.y
    los_lat = call_distance(TargetP, position)[1]

    # LOS heading(Desired heading angle)
    los_heading = atan2((los_lat), (los_lon))

    if los_heading < -pi:
        los_heading += pi
    elif los_heading > pi:
        los_heading -= pi

    if debug:
        print('Alpha: ', alpha)
        print('los_s: ', los_s)
        print('los_e: ', los_e)
        print('los_delta: ', los_delta)
        print('lat_proj: ', lat_proj)
        print('lon_proj: ', lon_proj)
        print('los_lat: ', los_lat)
        print('los_lon: ', los_lon)
        print('los_heading: ', los_heading * 180 / pi)

    # Return LOS heading angle and cross track error
    return np.array([los_heading, los_e])

if __name__ == '__main__':
  P1 = np.array([5050.710799, 00044.755897])
  P2 = np.array([5050.720397, 1044.759597])
  P3 = np.array([5050.732397, 00044.755897])
  wp_degress = DMM_to_DEG([5050.732397, 1044.755897])



