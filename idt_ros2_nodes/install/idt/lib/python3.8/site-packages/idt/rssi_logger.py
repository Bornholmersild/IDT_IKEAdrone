#!/usr/bin/env python3

# Combined Node for Logging RSSI, Remote RSSI, and Position
# SDU UAS Center
# University of Southern Denmark
# 2024-11-15 Kjeld Jensen and OpenAI GPT-4

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from datetime import datetime

class CombinedLogger(Node):
    def __init__(self):
        super().__init__('combined_logger')

        # Subscribers
        self.sub_global_pos = self.create_subscription(
            NavSatFix, 
            '/mavros/global_position/global', 
            self.on_global_pos_msg, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.sub_global_pos_raw = self.create_subscription(
            NavSatFix, 
            '/mavros/global_position/raw/fix', 
            self.on_global_pos_raw_msg, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.sub_diagnostics = self.create_subscription(
            DiagnosticArray, 
            '/diagnostics', 
            self.on_diagnostics_msg, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Variables to store the latest values
        self.latest_position = None
        self.latest_raw_position = None
        self.latest_rssi = None
        self.latest_remote_rssi = None

        # Timer for periodic logging
        self.timer = self.create_timer(1.0, self.log_combined_data)

        # File for logging
        self.log_file = "combined_logger.log"

        self.get_logger().info("CombinedLogger node started...")
        self._init_log_file()

    def _init_log_file(self):
        # Initialize log file and write a header
        with open(self.log_file, "a") as f:
            f.write(f"{'=' * 40}\n")
            f.write(f"Logging started: {datetime.now().isoformat()}\n")
            f.write(f"{'=' * 40}\n")

    def on_global_pos_msg(self, msg):
        self.latest_position = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().debug(f"Updated position: {self.latest_position}")

    def on_global_pos_raw_msg(self, msg):
        self.latest_raw_position = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().debug(f"Updated raw position: {self.latest_raw_position}")

    def on_diagnostics_msg(self, msg):
        for status in msg.status:
            if status.name == "mavros: 3DR Radio":
                for value in status.values:
                    if value.key == "RSSI (dBm)":
                        self.latest_rssi = value.value
                        self.get_logger().debug(f"Updated local RSSI: {self.latest_rssi}")
                    elif value.key == "Remote RSSI (dBm)":
                        self.latest_remote_rssi = value.value
                        self.get_logger().debug(f"Updated remote RSSI: {self.latest_remote_rssi}")

    def log_combined_data(self):
        # Consolidate and log the latest values
        if self.latest_position:
            pos_str = f"Position: {self.latest_position[0]:.6f}, {self.latest_position[1]:.6f}, {self.latest_position[2]:.2f} m"
        else:
            pos_str = "Position: N/A"

        if self.latest_raw_position:
            raw_pos_str = f"Raw Position: {self.latest_raw_position[0]:.6f}, {self.latest_raw_position[1]:.6f}, {self.latest_raw_position[2]:.2f} m"
        else:
            raw_pos_str = "Raw Position: N/A"

        rssi_str = f"RSSI: {self.latest_rssi}" if self.latest_rssi else "RSSI: N/A"
        remote_rssi_str = f"Remote RSSI: {self.latest_remote_rssi}" if self.latest_remote_rssi else "Remote RSSI: N/A"

        log_message = f"{datetime.now().isoformat()} | {pos_str} | {raw_pos_str} | {rssi_str} | {remote_rssi_str}"

        # Log to console
        self.get_logger().info(log_message)

        # Append to file
        with open(self.log_file, "a") as f:
            f.write(log_message + "\n")

def main(args=None):
    rclpy.init(args=args)
    combined_logger = CombinedLogger()
    rclpy.spin(combined_logger)

    # Shutdown
    combined_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
