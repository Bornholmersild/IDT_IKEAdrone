#!/usr/bin/env python3

"""
Author: Oscar Bowen Schofield (obs@sdu.dk)
Author Date: 2023-09-11

Description:
This node converts a text file into waypoint mission and uploads it to the drone.

to run the node, use the command "ros2 run idt upload_mission.py <file_location>"
Where <file_location> signifies the path to the .txt or .plan file

If using a text file, it should have the waypoint format: 
Latitude, Longitude, Altitude

For .plan files, the code only supports the following commands:
16 (waypoint)
20 (takeoff)
22 (return to home)

Hold times are default to 1 second, and can be changed on line 35

"""

import sys
import json
from asyncio import Future

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush

WAYPOINT_HOLD = 2.0 # seconds to pause at each waypoint

class RosNodeClass(Node):
    ''' Basic Class for a ROS node in rclpy '''
    def __init__(self, filename):
        super().__init__("get_global_pos")
        self.mission_filename = filename
        self.future = Future()
        self.get_logger().info("Upload Node Start...")

        self.waypoint_list = []
        self.takeoff_alt = 5.0  # default takeoff altitude

        self.get_logger().info(f'read_filename: {filename}')

        if filename.endswith(".plan"):
            permitted_commands = [16, 22]
            # assume the file is a QGC plan file, so read as a json file
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
                i=0 # waypoint sequence number
                if 'mission' in data and 'items' in data['mission']:
                    for waypoint in data['mission']['items']:
                        # read the lat, long and alt from the waypoint
                        line = f"{waypoint['params'][4]},\
                            {waypoint['params'][5]},\
                            {waypoint['params'][6]}"
                        # check if the command is valid (takeoff, waypoint)
                        if waypoint['command'] in permitted_commands:
                            self.waypoint_list.append(line)
                            print(self.waypoint_list[i])
                            i+=1
        else:
            # assume the file is using the Latitude, longitude, and altitude format
            # read the file for mission
            with open(filename, "r", encoding='utf-8') as f:
                for i, line in enumerate(f):
                    self.waypoint_list.append(line)
                    print(self.waypoint_list[i])

        # services
        self.waypoint_push_client = self.create_client(
            WaypointPush, "/mavros/mission/push"
        )
        while not self.waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Mission Upload service...")
        self.get_logger().info("Upload Service found")

    def generate_mission(self, hold_time=1.0):
        ''' Generates a MAVLink mission '''
        mission = []
        sequence_number = 0
        # generate a list of waypoints
        for coordinate in self.waypoint_list:
            # split the comma-separated coordinates into a list of floats
            coordinate_split = coordinate.split(",")

            waypoint_item = Waypoint()
            waypoint_item.frame = 3  # Global (relative to home) frame
            waypoint_item.autocontinue = True

            waypoint_item.x_lat = float(coordinate_split[0])
            waypoint_item.y_long = float(coordinate_split[1])

            if self.waypoint_list.index(coordinate) == 0:
                # assume the first waypoint is the takeoff position
                waypoint_item.z_alt = float(self.takeoff_alt)  # first takeoff altitude
                waypoint_item.command = 22  #  NAV_TAKEOFF
                waypoint_item.is_current = False
            else:
                waypoint_item.z_alt = float(coordinate_split[2])
                waypoint_item.command = 16  #  NAV_WAYPOINT
                waypoint_item.is_current = False

            waypoint_item.param1 = hold_time  # Hold time (seconds)
            waypoint_item.param2 = 0.0  # Acceptance radius
            waypoint_item.param3 = 0.0  # Yaw angle
            waypoint_item.param4 = float("nan")

            self.get_logger().info(f"Generated waypoint {sequence_number}: {waypoint_item}")
            mission.append(waypoint_item)
            sequence_number += 1

        # # Lastly, add a Return to Home command
        rth_waypoint = Waypoint()
        rth_waypoint.frame = 2
        rth_waypoint.is_current = False
        rth_waypoint.autocontinue = True
        rth_waypoint.command = 20  #  NAV_RETURN_TO_LAUNCH
        rth_waypoint.param1 = 0.0  # Hold time
        rth_waypoint.param2 = 0.0  # Acceptance radius
        rth_waypoint.param3 = 0.0  # Yaw angle
        rth_waypoint.param4 = 0.0
        mission.append(rth_waypoint)
        self.get_logger().info(f"Generated waypoint {sequence_number}: {rth_waypoint}")


        # generate service request
        upload_request = WaypointPush.Request()
        upload_request.start_index = 0
        upload_request.waypoints = mission

        # call the service asynchronously
        mission_callback = self.waypoint_push_client.call_async(upload_request)
        mission_callback.add_done_callback(self.waypoint_push_callback)

    def waypoint_push_callback(self, future):
        ''' Waits for a response from the service, the prints success message'''
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"{response.wp_transfered} waypoints uploaded successfully!"
                )
                self.get_logger().info("node will now close.")
                # set trigger to close the node
                self.future.set_result(True)
            else:
                self.get_logger().error(
                    f"Failed to upload waypoints. {response.wp_transfered}"
                )

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main():
    ''' the main call function '''

    if len(sys.argv) != 2:
        print(
            "Invalid Call \n Usage:  ros2 run idt upload_mission <waypoints_filename.txt>"
        )
        sys.exit()

    file_path = sys.argv[
        1
    ]  ## filepath to mission waypoint, input when running from terminal

    rclpy.init(args=None)

    ros_node = RosNodeClass(file_path)
    ros_node.generate_mission(WAYPOINT_HOLD)

    # this waits to hear for a True command from the asyncio within the node
    rclpy.spin_until_future_complete(ros_node, ros_node.future)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Call, Use:  ros2 run <package name> <node.py> <path>")
        sys.exit()
    main()
