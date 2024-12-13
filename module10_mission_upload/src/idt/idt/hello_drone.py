import math
import rclpy

from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import State


class ros_node_class(Node):
    def __init__(self):
        super().__init__('idt')

        self.msg_system_state = State()
        self.msg_fcu_orientation = Vector3()
        # publishers

        # subscribers

        # system state
        self.sub_system_state = self.create_subscription(
            State,
            '/mavros/state',
            self.on_system_state_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # local position pose
        self.sub_local_pos_pose = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.on_local_pos_pose_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.get_logger().warn('Hello Drone!')

        # timers
        self.timer = self.create_timer(1.0, self.timer_update)

    def on_system_state_msg(self, msg):
        self.msg_system_state = msg

    def on_local_pos_pose_msg(self, msg):
        # convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # store for use elsewhere
        self.msg_fcu_orientation.x = math.degrees(roll)
        self.msg_fcu_orientation.y = math.degrees(pitch)
        self.msg_fcu_orientation.z = math.degrees(yaw)

    def quaternion_to_euler(self, q_x, q_y, q_z, q_w):
        """Convert quaternion to euler angles""" 
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
        cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q_w * q_y - q_z * q_x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def timer_update(self):
        """
        Callback function for the timer. 
        Prints the FCU orientation and system state every second.""" 

        # Runs every second
        self.get_logger().info(
            f"System state: {self.msg_system_state.mode}, "
            f"roll: {self.msg_fcu_orientation.x:.2f}, "
            f"pitch: {self.msg_fcu_orientation.y:.2f}, "
            f"yaw: {self.msg_fcu_orientation.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    ros_node = ros_node_class()
    rclpy.spin(ros_node)

    ros_node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
