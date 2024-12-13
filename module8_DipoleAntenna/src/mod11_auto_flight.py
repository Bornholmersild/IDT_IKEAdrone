from range_test import *
from be_the_drone import *
from course_materials import hermite
from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

if __name__ == '__main__':
    
    # LOAD DATA
    lat, lon, alt, rssi, r_rssi = load_antenna_data("first_flight.log")

    # Conversion from geoid height to mean sea level
    for i in range(len(alt)):
        alt[i] = alt[i] - _egm96.height(lat[i], lon[i])

    geo_waypoints = [lat, lon, alt]	

    # INSTANCE FOR UTMCONV: TO CONVERT TO UTM AND SET REFERENCE COORDINATE
    us = utmconv()
    e, n, hemisphere, zone = convert_geodetic_to_utm(lat, lon, us)
    utm_waypoints = [e, n]	

    ###
    # RSSI Performance
    #
    # CALCULATE DISTANCE BETWEEN UTM COORDINATE
    dis_without_dipole = calculate_distance(e, n)

    # PLOT PATH AND 
    plt.plot(e, n)
    plt.axis='equal'
    plt.title("Captured rute")  # Add a title
    plt.xlabel("e (x-axis)")  # Label for the x-axis
    plt.ylabel("n (y-axis)")  # Label for the y-axis
    #plt.savefig("rute_plan_NG.png")


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
    #plt.savefig('rute_rssi_values_NG.png')
    #plt.show()
    
    ###
    # Simplify path
    ###
    trk = RouteSimplifier(geo_waypoints, utm_waypoints)
    lat_smw, lon_smw, alt_smw = trk.simplify_max_waypoints(25)
    trk.utm_plot(lat_smw, lon_smw, True ,'Maximum waypoint allowed')

    #lat_sdd, lon_sdd = trk.simplify_distance_deviation(0.000001)
    #e_sdd, n_sdd = trk.utm_plot(lat_sdd, lon_sdd, True ,'Minimize distance deviation from track using rdq')

    dda = DataConversion()

    e, n, hemisphere, zone = dda.geodetic_to_utm(lat_smw, lon_smw)
    e, n = dda.utm_reference_coordinates(e, n)
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Plot the surface
    ax.plot(e, n, alt_smw)

    # Labels and title
    ax.set_xlabel('e (x-axis)')
    ax.set_ylabel('n (y-axis)')
    ax.set_zlabel('Altitude (z-axis)')
    ax.set_title('Simplified 3D Route Plan')
    #plt.show()
    

    ### 
    # Fixed wing
    ###
    # Initialize the spline generator
    spline = hermite.cubic_hermite_spline()

    # Combine UTM points into a list of 2D points
    points = list(zip(e, n))

    # Initialize the spline generator
    spline = hermite.cubic_hermite_spline()

    # Calculate tangents
    k = 0.5  # Scaling factor for tangent length
    tangents = spline.calculate_tangents(points, k)

    # Generate the smooth curve
    smooth_curve = []
    steps = len(lat_smw)  # Number of points per segment
    for i in range(len(points)):
        p1 = points[i]
        p2 = points[(i + 1) % len(points)]  # Wrap around
        t1 = tangents[i]
        t2 = tangents[(i + 1) % len(points)]  # Wrap around
        smooth_curve += spline.goto_wpt(p1, t1, p2, t2, steps)

    # Visualization
    fixed_e, fixed_n = zip(*smooth_curve)
    plt.plot(fixed_e, fixed_n, label="Smoothed Curve")
    #plt.plot(e, n, color='red', label="Original Points")
    plt.legend()
    plt.axis = 'equal'
    plt.title("Smoothed Curve Using Cubic Hermite Splines")
    plt.show()

    fixed_lat, fixed_lon = dda.utm_to_geodetic(fixed_e, fixed_n, hemisphere, zone)

    # File for saveing waypoints
    waypoint_file = "waypoints_logger.txt"
    save_waypoints(waypoint_file, fixed_lat, fixed_lon, alt_smw)
        