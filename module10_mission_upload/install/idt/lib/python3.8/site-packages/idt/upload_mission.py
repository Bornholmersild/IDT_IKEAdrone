import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush

"""
Author: Oscar Bowen Schofield (obs@sdu.dk)
Author Date: 2023-09-11

Description:
This node converts a text file into waypoint mission and uploads it to the drone.

to run the node, use the command "ros2 run idt upload_mission.py <file_location>"
Where <file_location> signifies the path to the text file

The textfile should have the waypoint entries 
Latitude, Longitude, Altitude, Heading

Note: Heading is optional, leaving empty will result in transion heading being used

"""

class ros_node_class(Node):
    def __init__(self, filename):
        super().__init__('get_global_pos')
        self.get_logger().info('Upload Node Start...')

        self.waypoint_list = []
        self.takeoff_alt = 5.0 # default takeoff altitude

        self.get_logger().info('read_filename: %s' % filename)
        # read the file for mission
        with open(filename, 'r') as f:
            for i, line in enumerate(f):
                self.waypoint_list.append(line)
                print(self.waypoint_list[i])
                       
        # services
        self.waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        while not self.waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Mission Upload service...')
        self.get_logger().info('Upload Service found')

    def generate_mission(self):
        mission = []
        self.sequence_number = 0
        # generate a list of waypoints 
        for coordinate in self.waypoint_list:
            # split the comma-separated coordinates into a list of floats
            coordinate_split = coordinate.split(',')

            waypoint_item = Waypoint()
            waypoint_item.frame = 3  # Global (relative to home) frame
            waypoint_item.autocontinue = True

            waypoint_item.x_lat = float(coordinate_split[0])
            waypoint_item.y_long = float(coordinate_split[1])

            if self.waypoint_list.index(coordinate) == 0:
                # assume the first waypoint is the takeoff position
                waypoint_item.z_alt = float(self.takeoff_alt) # first takeoff altitude
                waypoint_item.command =  22 #  NAV_TAKEOFF 
                waypoint_item.is_current = False
            else:    
                waypoint_item.z_alt = float(coordinate_split[2])
                waypoint_item.command = 16  #  NAV_WAYPOINT
                waypoint_item.is_current = False


            waypoint_item.param1 = 1.0 # Hold time (seconds)
            waypoint_item.param2 = 0. # Acceptance radius
            waypoint_item.param3 = 0. # Yaw angle 
            waypoint_item.param4 = float('nan')

            mission.append(waypoint_item)

            self.get_logger().info(f'Generated waypoint: {waypoint_item}')

            self.sequence_number += 1
        # # Lastly, add a Return to Home command 
        rth_waypoint = Waypoint()
        rth_waypoint.frame = 2
        rth_waypoint.is_current = False
        rth_waypoint.autocontinue = True
        rth_waypoint.command = 20 #  NAV_RETURN_TO_LAUNCH
        rth_waypoint.param1 = 0. # Hold time
        rth_waypoint.param2 = 0. # Acceptance radius
        rth_waypoint.param3 = 0. # Yaw angle
        rth_waypoint.param4 = 0.0
        mission.append(rth_waypoint)  

        # generate service request
        upload_request = WaypointPush.Request()
        upload_request.start_index = 0
        upload_request.waypoints = mission
    
        # call the service asynchronously
        self.future = self.waypoint_push_client.call_async(upload_request)
        self.future.add_done_callback(self.waypoint_push_callback)
        rclpy.spin_until_future_complete(self, self.future)

    def waypoint_push_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{response.wp_transfered} waypoints uploaded successfully!')
                self.get_logger().info('node will now close.')
                self.destroy_node()
                rclpy.shutdown()
            else:
                self.get_logger().error(f'Failed to upload waypoints. {response.wp_transfered}')

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}') 



def main():
    
    if(len(sys.argv) != 2):
        print("Invalid Call \n Usage:  ros2 run idt upload_mission <waypoints_filename.txt>")
        sys.exit()

    file_path = sys.argv[1] ## filepath to mission waypoint, input when running from terminal 

    rclpy.init(args=None)

    ros_node = ros_node_class(file_path)
    ros_node.generate_mission()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if(len(sys.argv) != 2):
        print("Invalid Call, Use:  ros2 run <package name> <node.py> <path>")
        sys.exit()
    main()
