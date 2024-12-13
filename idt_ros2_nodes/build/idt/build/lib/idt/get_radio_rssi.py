#!/usr/bin/env python3

# Introduction to Drone Technology IDT
# SDU UAS Center
# University of Southern Denmark
# 2024-11-13 Kjeld Jensen kjen@sdu.dk First version

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from diagnostic_msgs.msg import DiagnosticArray

class RSSIListener(Node):
    def __init__(self):
        super().__init__('get_radio_rssi')
        # Subscribe to the /diagnostics topic
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.on_diagnostics,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.get_logger().info("get_radio_rssi node Started...")

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == "mavros: 3DR Radio":
                for value in status.values:
                    if value.key == "RSSI (dBm)":
                        rssi = value.value
                        self.get_logger().info(f"Local RSSI (dBm): {rssi}")
                    elif value.key == "Remote RSSI (dBm)":
                        remote_rssi = value.value
                        self.get_logger().info(f"Remote RSSI (dbm): {remote_rssi}")

def main(args=None):
    rclpy.init(args=args)
    rssi_listener = RSSIListener()
    rclpy.spin(rssi_listener)

    # Destroy the node explicitly
    rssi_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

