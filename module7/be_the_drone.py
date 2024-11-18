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
	
	def geodetic_to_utm(self, lat, lon):
		e, n = [], []
		for x, y in zip(lat, lon):
			hemisphere, zone, letter, e_, n_ = self.uc.geodetic_to_utm(x, y)
			e.append(e_)
			n.append(n_)
		
		self.start_utm = [e[0], n[0]]

		return e, n, hemisphere, zone
	
	def utm_reference_coordinates(self, e, n):
		'''
		Subtract starting coordinates from all coordinates
		'''
		e_adj = [x - self.start_utm[0] for x in e]
		n_adj = [y - self.start_utm[1] for y in n]

		return e_adj, n_adj
	
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
	def __init__(self, geo_waypoints, utm_waypoints):
		"""
		Initialize with a list of waypoints.
		:param waypoints: lists of list (lat, lon, alt)
		"""
		self.geo_pnt = geo_waypoints  				# Store the original list of waypoints
		self.utm_pnt = utm_waypoints
		self.simp_pnt = []  					# This will hold the simplified path
		self.DataConv_class = DataConversion()

	def simplify_max_waypoints(self, max_waypoints):
		"""
		Simplify the route by reducing the number of waypoints to a maximum count.
		:param max_waypoints: Maximum amount of data points
		""" 
		
		indices = np.linspace(0, len(self.geo_pnt[0]) - 1, max_waypoints, dtype=int)		# Evenly spaced indices
		simplified_arr = []

		for sublist in np.array(self.geo_pnt):					# Append for all waypoints
			simplified_arr.append(sublist[indices])
		
		return simplified_arr

	def simplify_distance_deviation(self, epsilon):
		"""
		Simplify the route by minimizing the distance deviation from the original track.
		:param epsilon: Maximum allowed distance deviation 
		"""		
		arr = np.array([self.geo_pnt[0], self.geo_pnt[1]]).T		# Convert to np.array for rdp

		simplified_arr = rdp(arr, epsilon = epsilon)			# Simplify the array using the RDP algorithm
		
		return np.array(simplified_arr).T  			# Convert back to numpy array

	def simplify_bearing_angle_deviation(self, e_sdd, n_sdd, bearing_epsilon):
		"""
		Simplify the route by minimizing bearing angle deviation.
		Reuse results from 'simplify_distance_deviation'
		:param bearing_epsilon: Max bearing deviation
		"""
		e_trk = []
		n_trk = []
		e_trk.append(e_sdd[0])		# Append starting coordinate
		n_trk.append(n_sdd[0])

		for i in range(1, len(e_sdd) - 1):
			pnt_a = (e_sdd[i - 1], n_sdd[i - 1])  	# Previous point
			pnt_b = (e_sdd[i], 	   n_sdd[i])  		# Current point
			pnt_c = (e_sdd[i + 1], n_sdd[i + 1])  	# Next point

			is_within = self.is_segment_within_angle(pnt_a, pnt_b, pnt_c, bearing_epsilon)

			if (is_within):
				e_trk.append(pnt_b[0])
				n_trk.append(pnt_b[1])

		e_trk.append(e_sdd[-1])		# Append starting coordinate
		n_trk.append(n_trk[-1])

		return e_trk, n_trk

	def calculate_bearing(self, point1, point2):
		"""
		Calculate the bearing angle (azimuth) between two points in 2D space.
		:param point1: Tuple (x1, y1)
		:param point2: Tuple (x2, y2)
		
		:return: azimuth in degrees
		"""
		x1, y1 = point1
		x2, y2 = point2

		# Direction vector
		dx = x2 - x1
		dy = y2 - y1

		# Azimuth angle (in XY-plane)
		azimuth = math.degrees(math.atan2(dy, dx))

		return azimuth

	def is_segment_within_angle(self, point1, point2, point3, max_azimuth_epsilon):
		"""
		Check if the angular deviation between two consecutive segments is within the allowable thresholds.
		:param point1: Tuple (x1, y1) - Start of first segment
		:param point2: Tuple (x2, y2) - End of first segment and start of second segment
		:param point3: Tuple (x3, y3) - End of second segment
		:param max_azimuth_epsilon: Maximum allowable azimuth deviation (degrees)

		:return: True if within allowable angular deviation, False otherwise
		"""
		# Calculate bearings for the two segments
		azimuth1 = self.calculate_bearing(point1, point2)
		azimuth2 = self.calculate_bearing(point2, point3)

		# Calculate angular differences
		azimuth_diff = abs(azimuth2 - azimuth1)

		# Normalize azimuth difference (e.g., handle wraparound at 360 degrees)
		if azimuth_diff > 180:
			azimuth_diff = 360 - azimuth_diff

		print(f"Azimuth_diff: {azimuth_diff:.2f}°    -   {azimuth_diff <= max_azimuth_epsilon}")

		# Check if differences exceed the thresholds
		return azimuth_diff <= max_azimuth_epsilon

	def utm_plot(self, lat, lon, plot_figure=True, plot_title=''):
		'''
		Convert geodetic coordinates to utm and make reference to starting point
		param: lat, lon is geodetic coordinates
		type: List
		param: Plot figure and set title text
		
		return: A list of east and north coordinates
		'''
		utm_coor = self.DataConv_class.geodetic_to_utm(lat, lon)						# Obtain utm coordinates [0] = east, [1] = north
		e, n = self.DataConv_class.utm_reference_coordinates(utm_coor[0], utm_coor[1])	# Make reference to starting point
		
		if plot_figure:
			self.plot_track([e, n], plot_title)

		return e, n

	def plot_track(self, simp_pnt, title_name):
		'''
		Plot original vs. simplified graph
		param: simp_pnt should be 
		'''
		fig = plt.figure()
		ax = fig.add_subplot(1, 1, 1)  # Add a single subplot to the figure

		# Plot the paths
		ax.plot(self.utm_pnt[0], self.utm_pnt[1], 'o-', label=f'Original Path - n = {len(self.utm_pnt[0])}')
		ax.plot(simp_pnt[0], simp_pnt[1], 'r-', label=f'Simplified Path - n = {len(simp_pnt[0])}')

		# Set labels, title, and legend
		ax.set_xlabel('Eastings')
		ax.set_ylabel('Northings')
		ax.set_title('Original and Simplified Paths')
		ax.legend()
		ax.set_title(title_name)
		ax.axis('equal')

		# Display the plot
		plt.show()

	def bearing_plot(self, rdp_pnt, bearing_pnt, title_name):
		'''
		Plot original and rdp path
		Plot rdp and bearing angle path
		'''
		# Create a figure with 1 row and 2 columns of subplots
		fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

		# First subplot: Original Path only
		ax1.plot(self.utm_pnt[0], self.utm_pnt[1], 'o-', label=f'Original Path - n = {len(self.utm_pnt[0])}')
		ax1.plot(rdp_pnt[0], rdp_pnt[1], 'r-', label=f'Simplified Path after rdp - n = {len(rdp_pnt[0])}')
		ax1.set_xlabel('Eastings')
		ax1.set_ylabel('Northings')
		ax1.set_title('Original and simplified path after rdp')
		ax1.legend(loc='upper right')
		ax1.axis('equal')

		# Second subplot: Original and Simplified Paths
		ax2.plot(rdp_pnt[0], rdp_pnt[1], 'ro-', label=f'rdp path - n = {len(rdp_pnt[0])}')
		ax2.plot(bearing_pnt[0], bearing_pnt[1], 'g-', label=f'Simplified Path after bearing angle - n = {len(bearing_pnt[0])}')
		ax2.set_xlabel('Eastings')
		ax2.set_ylabel('Northings')
		ax2.set_title(title_name)
		ax2.legend(loc='upper right')
		ax2.axis('equal')

		# Adjust layout for better spacing
		plt.tight_layout()

		# Display the plots
		plt.show()


def main():

	# Example set of points (x, y)
	points = [(1, 2), (3, 4), (5, 6), (7, 8), (-1, -2), (-3, -4)]

	# Angle threshold in radians (e.g., remove points where the angle exceeds 45 degrees)
	angle_threshold = np.radians(45)  # 45 degrees in radians

	# Function to filter points based on angle threshold
	def filter_points(points, angle_threshold):
		filtered_points = []
		for x, y in points:
			angle = np.arctan2(y, x)  # Calculate angle
			if abs(angle) <= angle_threshold:  # Only include points within the angle threshold
				filtered_points.append((x, y))
		return filtered_points

	# Get the filtered points
	filtered_points = filter_points(points, angle_threshold)
	print("Filtered Points:", filtered_points)
	'''
	Exercise 4.2 - Convert coordinates to UTM
	'''
	utm = DataConversion()
	lat, lon, alt = utm.load_data("gnns_data.txt")							# Return [latitude, longitude, altitude]
	geo_waypoints = [lat, lon]												# Make a list of geodetic coordinates
	e, n, hemisphere, zone = utm.geodetic_to_utm(lat, lon)				# Obtain utm coordinates
	e_adj, n_adj = utm.utm_reference_coordinates(e, n)		# Generate path with outliers
	utm_waypoints = [e_adj, n_adj]											# A list of UTM coorinates
	
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
	trk = RouteSimplifier(geo_waypoints, utm_waypoints)

	# Exercise 4.4.1
	lat_smw, lon_smw = trk.simplify_max_waypoints(50)
	trk.utm_plot(lat_smw, lon_smw, True ,'Maximum waypoint allowed')
	
	# Exercise 4.4.2
	lat_sdd, lon_sdd = trk.simplify_distance_deviation(0.000001)
	e_sdd, n_sdd = trk.utm_plot(lat_sdd, lon_sdd, True ,'Minimize distance deviation from track using rdq')
	

	# Exercise 4.4.3
	max_azimuth = 45  # Maximum allowed azimuth deviation in degrees
	e_trk, n_trk = trk.simplify_bearing_angle_deviation(e_sdd, n_sdd, max_azimuth)
	trk.bearing_plot([e_sdd, n_sdd], [e_trk, n_trk], 'Minimize distance and bearing angle deviation from track')

	'''
	Exercise 4.5 - Convert to geographic coordinates.
	'''
	lat_back, lon_back = utm.utm_to_geodetic(e_adj, n_adj, hemisphere, zone)
	
	# Create the first figure
	fig = plt.figure(1)
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(lat_back, lon_back, label="lat_back, lon_back", color='r')
	ax.set_xlabel('Latitude')
	ax.set_ylabel('Longitude')
	ax.set_title("Figure 1: lat_back vs lat, lon_back vs lon")
	ax.legend()

	# Create the second figure
	fig1 = plt.figure(2)
	ax1 = fig1.add_subplot(111, projection='3d')
	ax1.plot(lat, lon, label="lat, lon", color='b')
	ax1.set_xlabel('Latitude')
	ax1.set_ylabel('Longitude')
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