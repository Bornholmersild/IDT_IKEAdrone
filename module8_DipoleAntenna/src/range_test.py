#!/usr/bin/env python3

import numpy as np
import re
import matplotlib.pyplot as plt
import sys
from utm import utmconv
from scipy.signal import butter, filtfilt           # butterworth filter

def load_antenna_data(filename):
    # Define a list to store the extracted data
    lat = []
    lon = []
    alt = []
    rssi = []
    r_rssi = []

    # Open the log file and read line by line
    with open(filename, "r") as file:
        for line in file:
            #if line.split()[10] == "N/A":
            #    lat.append(float(line.split()[3].rstrip(',')))
            #    lon.append(float(line.split()[4].rstrip(',')))
            
            # Extract latitude, longitude, RSSI and remote RSSI
            lat.append(float(line.split()[10].rstrip(',')))
            lon.append(float(line.split()[11].rstrip(',')))
            alt.append(float(line.split()[12].rstrip(',')))
            rssi.append(float(line.split()[16]))
            r_rssi.append(float(line.split()[20]))

    return lat, lon, alt, rssi, r_rssi

def save_waypoints(filename, lat, lon, alt):
        # Initialize log file and write a header
        with open(filename, "w") as f:
            for i in range(len(lat)-1):
                f.write(f"{lat[i]},")
                f.write(f"{lon[i]},")
                f.write(f"{alt[i]}\n")

def calculate_distance(x, y):
    # Contain the distance
    res = []
    for i in range(len(x)):        
        # Calculate the distance using the Euclidean formula
        if i == 0:
            dis = np.sqrt((x[i] - x[i])**2 + (y[i] - y[i])**2)
        else:
            dis = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2) + res[i - 1]

        res.append(dis)

    return res

def plot_rssi_vs_dis(distance, rssi, remote_rssi):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))  # Create 1 row and 2 columns of subplots
    
    # Plot RSSI vs Distance
    axes[0].plot(distance, rssi, label='RSSI')
    axes[0].set_title('RSSI vs Distance')
    axes[0].set_xlabel('Distance [m]')
    axes[0].set_ylabel('RSSI [dBm]')
    axes[0].grid(True)
    
    # Plot Remote RSSI vs Distance
    axes[1].plot(distance, remote_rssi, label='Remote RSSI', color='orange')
    axes[1].set_title('Remote RSSI vs Distance')
    axes[1].set_xlabel('Distance [m]')
    axes[1].set_ylabel('Remote RSSI [dBm]')
    axes[1].grid(True)
    
    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()

def convert_geodetic_to_utm(lat, lon, us):
    e, n = [], []
    st = False
    start_utm = []
    for x, y in zip(lat, lon):
        hemisphere, zone, letter, e_, n_ = us.geodetic_to_utm(x, y)
        e.append(e_)
        n.append(n_)

        if st == False:
            st = True
            start_utm.append(e_)
            start_utm.append(n_)


    e_adj = [x - start_utm[0] for x in e]
    n_adj = [y - start_utm[1] for y in n]

    return e_adj, n_adj, hemisphere, zone


    
if __name__ == '__main__':
    # LOAD DATA
    lat, lon, rssi, r_rssi = load_antenna_data("log_dipole_antenna.log")
    
    # INSTANCE FOR UTMCONV: TO CONVERT TO UTM AND SET REFERENCE COORDINATE
    us = utmconv()
    e, n, hemisphere, zone = convert_geodetic_to_utm(lat, lon, us)

    # CALCULATE DISTANCE BETWEEN UTM COORDINATE
    dis_without_dipole = calculate_distance(e, n)

    # PLOT PATH AND 
    plt.plot(e, n)
    plt.axis='equal'
    plt.title("Captured rute")  # Add a title
    plt.xlabel("e (x-axis)")  # Label for the x-axis
    plt.ylabel("n (y-axis)")  # Label for the y-axis
    #plt.savefig("path_issued_antenna.png")
    plt.savefig("path_dipole_antenna.png")

    # BUTTERWORTH FILTER 
    order = 2                   # Order of complexity. Between 1 and 4
    cutoff = 0.05               # Nyquist frequency is 0.5. Adjust to smooth the rssi

    # Design Butterworth filter
    b, a = butter(order, cutoff, btype='low', analog=False)
    smoothed_rssi = filtfilt(b, a, rssi)
    smoothed_r_rssi = filtfilt(b, a, rssi)

    # RSSI VS DISTANCE PLOT
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))  # Create 1 row and 2 columns of subplots

    # Plot RSSI vs Distance
    axes[0].plot(dis_without_dipole, smoothed_rssi , label='RSSI')
    axes[0].set_title('RSSI vs Distance')
    axes[0].set_xlabel('Distance [m]')
    axes[0].set_ylabel('RSSI [dBm]')
    axes[0].grid(True)
    
    # Plot Remote RSSI vs Distance
    axes[1].plot(dis_without_dipole, smoothed_r_rssi , label='Remote RSSI', color='orange')
    axes[1].set_title('Remote RSSI vs Distance')
    axes[1].set_xlabel('Distance [m]')
    axes[1].set_ylabel('Remote RSSI [dBm]')
    axes[1].grid(True)
    
    # Adjust layout
    plt.tight_layout()
    #plt.savefig('issued_antenna_rssi_plot.png')
    plt.savefig('dipole_antenna_rssi_plot.png')

