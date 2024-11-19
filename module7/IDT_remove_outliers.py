import numpy as np
import matplotlib.pyplot as plt 

def remove_outliers(utm_data):
    #create array with distance between each point
    #each point is an x and y coordinate in utm_data
    #the distance should be the absolute distance between each point

    distances = []
    for i in range(0, len(utm_data)-1):
        x1 = utm_data[i][0]
        y1 = utm_data[i][1]
        x2 = utm_data[i+1][0]
        y2 = utm_data[i+1][1]
        distance = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        distances.append(distance)
    

    #remove outliers using sliding window
    #window size is 5
    #if the distance between the point and the mean of the window is greater than 2 times the standard deviation, remove the point
    #the cleaned data is stored in data_cleaned
    
    data_cleaned = []
    window_size = 5
    for i in range(0, len(distances)-window_size):
        window = distances[i:i+window_size]
        mean = np.mean(window)
        std = np.std(window)
        if abs(distances[i+window_size] - mean) < 2*std:
            data_cleaned.append(utm_data[i+window_size])
    return data_cleaned

