#!/usr/bin/env python3

# import utmconv class
from utm import utmconv
import math
from statistics import mean, stdev
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import random
from rdp import *
import numpy as np

class DataConversion:
	def __init__(self):
		# instantiate utmconv class
		self.uc = utmconv()
		self.start_utm = []

	def load_data(self, file_path="path"):
		'''
		Load GNSS data in tuples
		:param file_path: Define path in the same folder or predefine own path
		'''
		lat, lon, alt = [], [], []
		if file_path == "path":
			#f = open("/home/nicklas/Documents/DAS_kandidate/Drone_technology/module7/gnns_data.txt","r")
			pass
		else:
			with open(file_path, "r") as f:
				lines = f.readlines()
				for line in lines:
					data = line.split(' ')
					lat.append(float(data[0]))
					lon.append(float(data[1]))
					alt.append(float(data[2]))
		
		return lat, lon, alt
	
	def geodetic_to_utm(self, lat, lon, alt):
		e, n = [], []
		for x, y in zip(lat, lon):
			hemisphere, zone, letter, e_, n_ = self.uc.geodetic_to_utm(x, y)
			e.append(e_)
			n.append(n_)
		
		self.start_utm = [e[0], n[0], alt[0]]

		return e, n, hemisphere, zone
	
	def utm_reference_coordinates(self, e, n, alt):
		'''
		Subtract starting coordinates from all coordinates
		'''
		e_adj = [x - self.start_utm[0] for x in e]
		n_adj = [y - self.start_utm[1] for y in n]
		alt_adj = [z - self.start_utm[2] for z in alt]

		return e_adj, n_adj, alt_adj
	
	def utm_to_geodetic(self, e_adj, n_adj, hemisphere, zone):
		'''
		Convert back into geodetic 
		'''
		if not self.start_utm:
			return False, "start_utm is empty"
		
		e_adj_back = []
		n_adj_back = []
		for x,y in zip(e_adj, n_adj):
			e_adj_back.append(x + self.start_utm[0])
			n_adj_back.append(y + self.start_utm[1])

		lat_back, lon_back = [], []
		for e, n in zip(e_adj, n_adj):
			lat, lon = self.uc.utm_to_geodetic(hemisphere, zone, e, n)
			lat_back.append(lat)
			lon_back.append(lon)

		return lat_back, lon_back

class RouteSimplifier:
	def __init__(self, waypoints):
		"""
		Initialize with a list of waypoints.
		:param waypoints: lists of list (lat, lon, alt)
		"""
		self.ori_pnt = waypoints  				# Store the original list of waypoints
		self.simp_pnt = []  					# This will hold the simplified path

	def simplify_max_waypoints(self, max_waypoints):
		"""
		Simplify the route by reducing the number of waypoints to a maximum count.
		:param max_waypoints: Maximum amount of data points
		""" 
		
		indices = np.linspace(0, len(self.ori_pnt[0]) - 1, max_waypoints, dtype=int)		# Evenly spaced indices
		simplified_arr = []

		for sublist in np.array(self.ori_pnt):					# Append for all waypoints
			simplified_arr.append(sublist[indices])
		
		return simplified_arr

	def simplify_distance_deviation(self, epsilon):
		"""
		Simplify the route by minimizing the distance deviation from the original track.
		:param epsilon: Maximum allowed distance deviation 
		"""		
		arr = np.array([self.ori_pnt[0], self.ori_pnt[1], self.ori_pnt[2]]).T		# Convert to np.array for rdp

		simplified_arr = rdp(arr, epsilon = epsilon)			# Simplify the array using the RDP algorithm
		
		return np.array(simplified_arr).T  			# Convert back to numpy array

	def simplify_distance_bearing_deviation(self, distance_epsilon, bearing_epsilon):
		"""
		Simplify the route by minimizing distance and bearing angle deviation.
		:param distance_epsilon: Max distance deviation
		:param bearing_epsilon: Max bearing deviation
		"""
		# Implement method

		# Utility methods for distance and bearing calculation can be added here

	def plot_track(self, ori_pnt, simp_pnt, title_name):
		# Plot the simplified array
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.plot(ori_pnt[0], ori_pnt[1], ori_pnt[2], 'o-', label = f'Original Path - n = {len(ori_pnt[0])}')
		ax.set_xlabel('Eastings')
		ax.set_ylabel('Northings')
		ax.set_zlabel('Altitude')
		ax.set_title(title_name)
		plt.legend()
		plt.axis = 'equal'
	
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.plot(ori_pnt[0], ori_pnt[1], ori_pnt[2], 'o-', label = f'Original Path - n = {len(ori_pnt[0])}')
		ax.plot(simp_pnt[0], simp_pnt[1], simp_pnt[2], 'rx-', label = f'Simplified Path - n = {len(simp_pnt[0])}')
		ax.set_xlabel('Eastings')
		ax.set_ylabel('Northings')
		ax.set_zlabel('Altitude')
		ax.set_title(title_name)
		plt.legend()
		plt.axis = 'equal'
		plt.show()

	def calculate_bearing_3d(self, point1, point2):
		"""
		Calculate the bearing angle (azimuth, elevation) between two points in 3D space.
		:param point1: Tuple (x1, y1, z1)
		:param point2: Tuple (x2, y2, z2)
		:return: (azimuth, elevation) in degrees
		"""
		x1, y1, z1 = point1
		x2, y2, z2 = point2

		# Direction vector
		dx = x2 - x1
		dy = y2 - y1
		dz = z2 - z1

		# Azimuth angle (in XY-plane)
		azimuth = math.degrees(math.atan2(dy, dx))

		# Elevation angle (angle above the XY-plane)
		elevation = math.degrees(math.atan2(dz, math.sqrt(dx**2 + dy**2)))

		return azimuth, elevation

	def is_segment_within_angle(self, point1, point2, point3, max_azimuth_epsilon, max_elevation_epsilon):
		"""
		Check if the angular deviation between two consecutive segments is within the allowable thresholds.
		:param point1: Tuple (x1, y1, z1) - Start of first segment
		:param point2: Tuple (x2, y2, z2) - End of first segment and start of second segment
		:param point3: Tuple (x3, y3, z3) - End of second segment
		:param max_azimuth_epsilon: Maximum allowable azimuth deviation (degrees)
		:param max_elevation_epsilon: Maximum allowable elevation deviation (degrees)
		:return: True if within allowable angular deviation, False otherwise
		"""
		# Calculate bearings for the two segments
		azimuth1, elevation1 = self.calculate_bearing_3d(point1, point2)
		azimuth2, elevation2 = self.calculate_bearing_3d(point2, point3)

		# Calculate angular differences
		azimuth_diff = abs(azimuth2 - azimuth1)
		elevation_diff = abs(elevation2 - elevation1)

		# Normalize azimuth difference (e.g., handle wraparound at 360 degrees)
		if azimuth_diff > 180:
			azimuth_diff = 360 - azimuth_diff

		print(f"Azimuth_diff: {azimuth_diff:.2f}°, Elevation_diff: {elevation_diff:.2f}°")

		# Check if differences exceed the thresholds
		return azimuth_diff <= max_azimuth_epsilon and elevation_diff <= max_elevation_epsilon



def main():
	'''
	Exercise 4.2 - Convert coordinates to UTM
	'''
	utm = DataConversion()
	lat, lon, alt = utm.load_data("gnns_data.txt")							# Return [latitude, longitude, altitude]
	waypoints = [lat, lon, alt]												# Make a list of geodetic coordinates
	e, n, hemisphere, zone = utm.geodetic_to_utm(lat, lon, alt)				# Obtain utm coordinates
	e_adj, n_adj, alt_adj = utm.utm_reference_coordinates(e, n, alt)		# Generate path with outliers
	
	
	''' Adding random noise for outliers
	for i in range(50):
		e_adj[random.randint(0,1100)] = e_adj[random.randint(0,1100)] + random.random()

	plt.plot(e_adj,n_adj,'r*')
	plt.axis(True)
	plt.axis('equal')
	plt.show()

	fig = plt.figure(1)
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(e_adj, n_adj, alt_adj, label="lat_back, lon_back, alt", color='r')
	ax.set_xlabel('Latitude')
	ax.set_ylabel('Longitude')
	ax.set_zlabel('Altitude')
	ax.set_title("Figure 1: lat_back vs lat, lon_back vs lon")
	ax.legend()
	plt.show()'''

	'''
	Exercise 4.3 - Remove outliers
	'''
	# REMOVE OUTLIERS FUNCTION CALL

	'''
	Exercise 4.4 - Simplify the track
		1. maximum waypoints allowed.
		2. minimize distance deviation from track.
		3. minimize distance and bearing angle deviation from track.
	'''
	trk = RouteSimplifier(waypoints)

	# Exercise 4.4.1
	lat_sim, lon_sim, alt_sim = trk.simplify_max_waypoints(50)
	e_sim, n_sim, hemisphere_sim, zone_sim = utm.geodetic_to_utm(lat_sim, lon_sim, alt_sim)				# Obtain utm coordinates
	e_sim_adj, n_sim_adj, alt_sim_adj = utm.utm_reference_coordinates(e_sim, n_sim, alt_sim)			# Generate path with outliers
	trk.plot_track([e_adj, n_adj, alt_adj], [e_sim_adj, n_sim_adj, alt_sim_adj], 'Maximum waypoints allowed')
	
	# Exercise 4.4.2
	lat_sim, lon_sim, alt_sim = trk.simplify_distance_deviation(0.000001)
	e_sim, n_sim, hemisphere_sim, zone_sim = utm.geodetic_to_utm(lat_sim, lon_sim, alt_sim)				# Obtain utm coordinates
	e_sim_adj, n_sim_adj, alt_sim_adj = utm.utm_reference_coordinates(e_sim, n_sim, alt_sim)			# Generate path with outliers
	trk.plot_track([e_adj, n_adj, alt_adj], [e_sim_adj, n_sim_adj, alt_sim_adj], 'Minimize distance deviation from track using rdq')
	

	# Exercise 4.4.3
	max_azimuth = 90  # Maximum allowed azimuth deviation in degrees
	max_elevation = 50  # Maximum allowed elevation deviation in degrees
	
	simp_track = []
	simp_track.append([e_sim_adj[0], n_sim_adj[0], alt_sim_adj[0]])

	for i in range(1, len(e_sim_adj) - 1):
		pnt_a = (e_sim_adj[i], n_sim_adj[i - 1], alt_sim_adj[i - 1])  	# Previous point
		pnt_b = (e_sim_adj[i], n_sim_adj[i], 	 alt_sim_adj[i])  		# Current point
		pnt_c = (e_sim_adj[i], n_sim_adj[i + 1], alt_sim_adj[i + 1])  	# Next point

		is_within = trk.is_segment_within_angle(pnt_a, pnt_b, pnt_c, max_azimuth, max_elevation)
		print(f"Segment is within allowed angle deviation: {is_within}")

		if (is_within):
			simp_track.append(pnt_b)

	simp_track.append([e_sim_adj[-1], n_sim_adj[-1], alt_sim_adj[-1]])
	e_sim_adj, n_sim_adj, alt_sim_adj = np.array(simp_track).T
	trk.plot_track([e_adj, n_adj, alt_adj], [e_sim_adj, n_sim_adj, alt_sim_adj], 'minimize distance and bearing angle deviation from track.')

	'''
	Exercise 4.5 - Convert to geographic coordinates.
	'''
	lat_back, lon_back = utm.utm_to_geodetic(e_adj, n_adj, hemisphere, zone)
	
	# Create the first figure
	fig = plt.figure(1)
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(lat_back, lon_back, alt, label="lat_back, lon_back, alt", color='r')
	ax.set_xlabel('Latitude')
	ax.set_ylabel('Longitude')
	ax.set_zlabel('Altitude')
	ax.set_title("Figure 1: lat_back vs lat, lon_back vs lon")
	ax.legend()

	# Create the second figure
	fig1 = plt.figure(2)
	ax1 = fig1.add_subplot(111, projection='3d')
	ax1.plot(lat, lon, alt, label="lat, lon, alt", color='b')
	ax1.set_xlabel('Latitude')
	ax1.set_ylabel('Longitude')
	ax1.set_zlabel('Altitude')
	ax1.set_title("Figure 2: lat vs lat_back, lon vs lon_back")
	ax1.legend()

	# Display the figures
	plt.show()

	'''
	Exercise 4.6 - Export as a route plan to QGroundControl software
	'''

	'''
	Exercise 4.7 - Fixed wing optimization
	'''
	

	
	

	
	

if __name__ ==  "__main__":
	main()