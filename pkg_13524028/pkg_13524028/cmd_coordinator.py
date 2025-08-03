#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdCoordinator(Node):
    def __init__(self):
        super().__init__('cmd_coordinator')
        # Declare parameters for topic names
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_type_topic', 'cmd_type')
        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel')

        # Get parameter values
        autonomous_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value
        
        # Subscriptions
        self.autonomous_sub = self.create_subscription(
            Twist,
            autonomous_topic,
            self.vel_callback,
            10)
        
        # Publishers to cmd_vel and cmd_type
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.cmd_type_pub = self.create_publisher(String, cmd_type_topic, 10)

        # Timer to publish periodically
        # Benar:
        self.timer = self.create_timer(0.2, self.publish_cmd)


    def vel_callback(self, msg):
        self.current_twist = msg

    def publish_cmd(self):
        # Publish cmd_vel
        self.cmd_vel_pub.publish(self.current_twist)
        cmd_type_msg = String()
        cmd_type_msg.data = "autonomous"
        self.cmd_type_pub.publish(cmd_type_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()