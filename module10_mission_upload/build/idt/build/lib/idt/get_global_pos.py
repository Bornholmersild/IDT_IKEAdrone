import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix

class ros_node_class(Node):
    def __init__(self):
        super().__init__('get_global_pos')

        # publishers

        # subscribers
        self.sub_global_pos = self.create_subscription(
            NavSatFix,                             # message type
            '/mavros/global_position/global',      # topic to subscribe to
            self.on_global_pos_msg,                # callback function
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.sub_global_pos_raw = self.create_subscription(
            NavSatFix, # message type
            '/mavros/global_position/raw/fix', # topic to subscribe to
            self.on_global_pos_raw_msg, # callback function 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # timers
        self.timer = self.create_timer(0.5, self.timer_update)
 
    def on_global_pos_msg(self, msg):
        self.get_logger().info('Got position: %.6f %.6f %.3f ' % (msg.latitude, msg.longitude, msg.altitude))

    def on_global_pos_raw_msg(self, msg):
        self.get_logger().info('Got raw position: %.6f %.6f %.3f ' % (msg.latitude, msg.longitude, msg.altitude))

    def timer_update(self):
        self.get_logger().info('Timer update')

def main(args=None):
    rclpy.init(args=args)
    ros_node = ros_node_class()
    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
