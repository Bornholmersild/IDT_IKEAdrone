1. To run, firstly build the ROS2 Workspace: colcon build

2. Source the Workspace: source /devel/setup.bash

3. run mavros with your telemetry plugged in
    >  ensure QGroundControl has SiK radio auto-connect disabled (see confluence)

ros2 run mavros mavros_node --ros-args --param fcu_url:=/dev/ttyACM0:57600 --param plugin_denylist:="[odometry]" --param gcs_url:=udp://:14540@127.0.0.1

4. in a new terminal, source the devel/setup.bash
5. run the node with: 
ros2 run idt upload_mission <path_to_waypoints.txt>

Note: the first waypoint is assumed to be your takeoff location, at which it will fly to 
5 meters in altitude before starting the flight.

Ensure that the waypoints are contained in the log file in the following format:

latitude, longitude, altitude
example: 55.4717713,10.325005,10.0

