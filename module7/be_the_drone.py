#!/usr/bin/env python3

# import utmconv class
from utm import utmconv
import math
from statistics import mean, stdev
import matplotlib.pyplot as plt
import random
from rdp import *
import numpy as np
from route_plan_exporter import RoutePlanExporter
from course_materials.hermite import cubic_hermite_spline

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

	def simplify_bearing_angle_deviation(self, lat_sdd, lon_sdd, bearing_epsilon):
		"""
		Simplify the route by minimizing bearing angle deviation.
		Reuse results from 'simplify_distance_deviation'
		:param bearing_epsilon: Max bearing deviation
		"""
		lat_trk = []
		lon_trk = []
		lat_trk.append(lat_sdd[0])		# Append starting coordinate
		lon_trk.append(lon_sdd[0])

		for i in range(1, len(lat_sdd) - 1):
			pnt_a = (lat_sdd[i - 1], lon_sdd[i - 1])  	# Previous point
			pnt_b = (lat_sdd[i], 	 lon_sdd[i])  		# Current point
			pnt_c = (lat_sdd[i + 1], lon_sdd[i + 1])  	# Next point

			# Calculate bearings for consecutive segments
			bearing1 = self.calculate_bearing(pnt_a, pnt_b)
			bearing2 = self.calculate_bearing(pnt_b, pnt_c)

			# Check angle difference
			if abs(bearing2 - bearing1) < bearing_epsilon:
				lat_trk.append(pnt_b[0])
				lon_trk.append(pnt_b[1])

		lat_trk.append(lat_sdd[-1])		# Append starting coordinate
		lon_trk.append(lon_sdd[-1])

		return lat_trk, lon_trk

	def calculate_bearing(self, point1, point2):
		"""
		Calculate the bearing angle (azimuth) between two points in 2D space.
		:param point1: List [lat1, lon1]
		:param point2: List [lat2, lon2]
		
		:return: azimuth in degrees
		"""
		lat1 = point1[0]
		lat2 = point2[0]
		lon1 = point1[1]
		lon2 = point2[1]

		delta_lon = math.radians(lon2 - lon1)
		lat1, lat2 = math.radians(lat1), math.radians(lat2)
		
		y = math.sin(delta_lon) * math.cos(lat2)
		x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
		return math.degrees(math.atan2(y, x))

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
		utm_coor = self.DataConv_class.geodetic_to_utm(bearing_pnt[0], bearing_pnt[1])	# Obtain utm coordinates [0] = east, [1] = north
		e, n = self.DataConv_class.utm_reference_coordinates(utm_coor[0], utm_coor[1])	# Make reference to starting point

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
		ax2.plot(e, n, 'g-', label=f'Simplified Path after bearing angle - n = {len(bearing_pnt[0])}')
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
	bearing_threshold = 90  # Maximum allowed azimuth deviation in degrees
	lat_trk, lon_trk = trk.simplify_bearing_angle_deviation(lat_sdd, lon_sdd, bearing_threshold)
	trk.bearing_plot([e_sdd, n_sdd], [lat_trk, lon_trk], 'Minimize distance and bearing angle deviation from track')

	'''
	Exercise 4.5 - Convert to geographic coordinates.
	'''
	lat_back, lon_back = utm.utm_to_geodetic(e_adj, n_adj, hemisphere, zone)

	fig = plt.figure()
	ax = fig.add_subplot(1, 1, 1)  # Add a single subplot to the figure

	# Plot the paths
	ax.plot(lat_back, lon_back, 'o-', label=f'Original Path')

	# Set labels, title, and legend
	ax.set_xlabel('Latitude')
	ax.set_ylabel('Longitude')
	ax.set_title('Convert back to geopraphic coordinates')
	ax.legend()
	ax.axis('equal')

	# Display the plot
	plt.show()

	'''
	Exercise 4.6 - Export as a route plan to QGroundControl software
	'''
	exporter = RoutePlanExporter()
	
	w_pnt = [lat_back, lon_back, alt]
	exporter.add_home_position(w_pnt[0][0], w_pnt[1][0], w_pnt[2][0])

	for i in range(len(w_pnt)):
		exporter.add_waypoint(w_pnt[0], w_pnt[1], w_pnt[2])

	# Exporting to a file
	exporter.export_to_file()

	'''
	Exercise 4.7 - Fixed wing optimization
	'''
	chs = cubic_hermite_spline()

	p1 = [0.0, 0.0]
	t1 = [0.0, 5.0]
	p2 = [2.0, 0.0]
	t2 = [0.0, 5.0]
	steps = 200
	rte = []
	rte.append(chs.goto_wpt (p1,t1,p2,t2,steps))

	p1 = [2.0, 0.0]
	t1 = [0.0, 5.0]
	p2 = [4.0, 0.0]
	t2 = [0.0, 5.0]
	rte.append(chs.goto_wpt (p1,t1,p2,t2,steps))
	print (rte)


	# plot route
	fig1 = plt.figure()
	rteT = list(zip(*rte))
	rte_plt = fig1.plot(rteT[:0],rteT[:1],'blue')

	fig1.title ('Route')
	fig1.axis('equal')
	fig1.xlabel('Easting [m]')
	fig1.ylabel('Northing [m]')
	plt.savefig ('route_plan.png')
	fig1.show()
	

	
	

	
	

if __name__ ==  "__main__":
	main()