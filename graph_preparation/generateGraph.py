# -*- coding: utf-8 -*-
"""
Created on Fri May 21 11:20:36 2021

@author: Arindam
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import seaborn as sns
import networkx as nx
import sympy as sp
import mpmath
from geopy.distance import geodesic
from geographiclib.geodesic import Geodesic

os.chdir("C:\\PhD courses\\Adv CPS\\project\\code")


#Unitary energy cost computation starts here    ****don't touch it****
mpmath.mp.dps = 5


# physic model for the drone's energy consumption. Do not touch it!
# It returns the energy required for flying distance meter
def get_energy(distance, payload_weight, drone_speed, wind_speed, wind_direction):

    # start calculations
    m_package = payload_weight
    m_drone = 7
    m_battery = 10

    num_rotors = 8
    diameter = 0.432

    # s_battery = 540000
    # delta = 0.5
    # f = 1.2

    pressure = 100726  # 50 meters above sea level
    R = 287.058
    temperature = 15 + 273.15  # 15 degrees in Kelvin
    rho = pressure / (R*temperature)

    g = 9.81

    # power efficiency
    eta = 0.7

    drag_coefficient_drone = 1.49
    drag_coefficient_battery = 1
    drag_coefficient_package = 2.2

    projected_area_drone = 0.224
    projected_area_battery = 0.015
    projected_area_package = 0.0929

    v_north = drone_speed - wind_speed*np.cos(np.deg2rad(wind_direction))
    v_east = - wind_speed*np.sin(np.deg2rad(wind_direction))
    v_air = np.sqrt(v_north**2 + v_east**2)

    # Drag force
    F_drag_drone = 0.5 * rho * (v_air**2) * drag_coefficient_drone * projected_area_drone
    F_drag_battery = 0.5 * rho * (v_air**2) * drag_coefficient_battery * projected_area_battery
    F_drag_package = 0.5 * rho * (v_air**2) * drag_coefficient_package * projected_area_package

    F_drag = F_drag_drone + F_drag_battery + F_drag_package

    alpha = np.arctan(F_drag / ((m_drone + m_battery + m_package)*g))

    # Thrust
    T = (m_drone + m_battery + m_package)*g + F_drag

    # # Power min hover
    # P_min_hover = (T**1.5) / (np.sqrt(0.5 * np.pi * num_rotors * (diameter**2) * rho))

    # v_i = Symbol('v_i')
    # f_0 = v_i - (2*T / (np.pi * num_rotors * (diameter**2) * rho * sp.sqrt((drone_speed*sp.cos(alpha))**2 + (drone_speed*sp.sin(alpha) + v_i)**2)))
    # induced_speed = float(nsolve(f_0, v_i, 5))
    # print(induced_speed)

    tmp_a = 2*T
    tmp_b = np.pi * num_rotors * (diameter**2) * rho
    tmp_c = (drone_speed*sp.cos(alpha))**2
    tmp_d = drone_speed*sp.sin(alpha)
    tmp_e = tmp_a / tmp_b

    coeff = [1, (2*tmp_d), (tmp_c+tmp_d**2), 0, -tmp_e**2]
    sol = np.roots(coeff)
    induced_speed = float(max(sol[np.isreal(sol)]).real)
    # print(induced_speed)

    # Power min to go forward
    P_min = T*(drone_speed*np.sin(alpha) + induced_speed)

    # expended power
    P = P_min / eta

    # energy efficiency of travel
    mu = P / drone_speed

    # Energy consumed
    E = mu * distance

    # # Range of a drone
    # R = (m_battery * s_battery * delta) / (e * f)

    return E/1000.


# this calculates the energy costs once, putting them on a table so that you save time.
# as you can see, you get the unitary costs (distance = 1)
# then, you vary the other parameters like:
#
#   distance = 1   <-- DO NOT TOUCH
#   payload_weights = [0, 7]
#   drone_speeds = [10, 20]
#   global_wind_speeds = [0, 5, 10, 15]
#   relative_wind_directions = [0, 45, 135, 180]   <-- DO NOT TOUCH
#
# it is important to say that you should not modify "distance" and "relative_wind_directions"
# however, you can modify the others, e.g.:
#
#   payload_weights = [0, 2, 4, 6]
#   drone_speeds = [5, 10, 15, 20]
#   global_wind_speeds = [0, 10, 20]
#
# keep this table as light as possible, so you won't affect the performance of your application
# use only the parameters that you'll actually use!
def compute_prefixes():
    distance = 1
    payload_weights = [0, 7]
    drone_speeds = [10, 20] #take 10
    global_wind_speeds = [0, 5, 10, 15] 
    relative_wind_directions = [0, 45, 135, 180]

    prefix = {}
    for p in payload_weights:
        for v in drone_speeds:
            for ws in global_wind_speeds:
                for wd in relative_wind_directions:
                    prefix[(p, v, ws, wd)] = get_energy(distance, p, v, ws, wd)

    return prefix
#Unitary energy cost computation ends here    ****don't touch it****



# we calculate the relative wind direction in classes, in order to reduce the calculations
# there are only 4 classes, i.e., 0, 45, 135, and 180 degrees (details on T-ITS paper)
def get_relative_wind(u, v, global_wind_direction):
    w_d = global_wind_direction
    drone_direction = directions[u][v]
    if drone_direction < 0:
        drone_direction = drone_direction + 360

    # relative wind direction
    r_wd = abs(drone_direction - w_d)

    if r_wd <= 45 or r_wd >= 315:
        r_wd = 0
    elif r_wd <= 90 or r_wd >= 270:
        r_wd = 45
    elif r_wd <= 135 or r_wd >= 225:
        r_wd = 135
    else:
        r_wd = 180

    return r_wd


# it returns the actual energy cost for each edge
#
# u and v are the two vertices
# the other parameters are clear
def compute_edge_weight(u, v, payload_weight, drone_speed, global_wind_speed, global_wind_direction):
    distance = distances[u][v]/100 #****AK::scaling down by 100 as to keep the distance in range 500m to 5km
    relative_wind_direction = get_relative_wind(u, v, global_wind_direction)
    weight = prefixes[(payload_weight, drone_speed, global_wind_speed, relative_wind_direction)] * distance/15 #****AK::scaling down wt by factor 15 just for computational simplicity

    # the energy cost
    return weight





#it return the bearing angle in degree between two real coordinates
#uses "from geographiclib.geodesic import Geodesic"
#azi1 returns angle measured from point 1
#azil2 returns angle measured from point 2
def get_bearing(lat1, lat2, long1, long2):
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    return brng



    
if __name__ == '__main__':
    # this is the "table" for the unitary energy costs.
    prefixes = compute_prefixes()  
    
    
    G = nx.read_gml('TataNld.gml.txt', label='id')
    n = len(G.nodes)
    m = len(G.edges)
    
    
    #for (x,y) in G.edges:
        #print(G._node[x]['Longitude'], G._node[x]['Latitude'])
        
        
    # distances and directions that will be filled once "create_graph" is performed
    distances = np.zeros((n, n))
    directions = np.zeros((n, n))
    
    # distance: D (m) x D (m) (m = meters)
    # this is a factor scale. The goal is to have, e.g., vertices belonging to a square of 2km x 2km
    # you can set this parameter as you wish
    D = 2000  
    
    #compute distance and direction
    for u, vdict in G.adjacency():
        for v in vdict:
            # distances among vertex u and vertex v
            #distances[u][v] = np.sqrt((G._node[u]['Longitude'] * D - G._node[v]['Longitude'] * D) ** 2 + (G._node[u]['Latitude'] * D - G._node[v]['Latitude'] * D) ** 2)
            ##Geo distance calculation from lat and long
            u_coordinate = (G._node[u]['Latitude'],G._node[u]['Longitude'])
            v_coordinate = (G._node[v]['Latitude'],G._node[v]['Longitude'])
            distances[u][v] = geodesic(u_coordinate, v_coordinate).m #in meters
            #print(u, v, G._node[v]['Latitude'], G._node[u]['Latitude'], G._node[v]['Longitude'], G._node[u]['Longitude'])
            # directions among vertex u and vertex v
            #directions[u][v] = int(np.rad2deg(float(sp.atan2(G._node[v]['Latitude'] - G._node[u]['Latitude'], G._node[v]['Longitude'] - G._node[u]['Longitude']))))   
            directions[u][v] = get_bearing(G._node[u]['Latitude'],G._node[v]['Latitude'],G._node[u]['Longitude'],G._node[v]['Longitude'])
            #print(u, v, distances[u][v], directions[u][v])
            
            
    ##define parameters: change it as required
    payload_weight = [7,0] #take 7 when going to customer. take 0 when coming back after delivery
    drone_speed = 10 #take 10 or 20
    global_wind_speed = [0,5,10,15] #options 0, 5, 10, 15
    global_wind_direction = [0,45,90,135,180] #any value between 0 and 180
    for p in payload_weight:
        for ws in global_wind_speed:
            for wd in global_wind_direction:
                fileName = 'TATA_p' + str(p)+'_ws'+str(ws)+'_wd'+str(wd)+'.txt'
                file1 = open(fileName,"w")
                for u, vdict in G.adjacency():
                    for v in vdict:
                        wt = compute_edge_weight(u, v, p, drone_speed, ws, wd)
                        #print(u, v, round(wt/10, 2)) #scaling down wt by factor 20 just for computational simplicity
                        str1 = "{} {} {}\n".format(u+1, v+1, round(wt, 2))
                        print(str1)
                        file1.write(str1)
                file1.close()
    