#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from queue import PriorityQueue
import time

class CmdCoordinator(Node):
    def __init__(self):
        super().__init__('cmd_coordinator')
        self.priority_map = {
            'keyboard': 1,
            'joy': 2,
            'autonomous': 3
        }
        self.cmd_queue = PriorityQueue()
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_type_topic', 'cmd_type')
        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel')
        self.declare_parameter('joy_vel_topic', 'joy_vel')
        self.declare_parameter('keyboard_vel_topic', 'keyboard_vel')

        autonomous_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_vel_topic').get_parameter_value().string_value
        keyboard_topic = self.get_parameter('keyboard_vel_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value
        
        self.keyboard_sub = self.create_subscription(
            Twist,
            keyboard_topic,
            self.keyboard_callback,
            10)
        self.joy_sub = self.create_subscription(
            Twist,
            joy_topic,
            self.joy_callback,
            10)
        self.autonomous_sub = self.create_subscription(
            Twist,
            autonomous_topic,
            self.autonomous_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.cmd_type_pub = self.create_publisher(String, cmd_type_topic, 10)

        self.timer = self.create_timer(0.2, self.publish_cmd)


    def keyboard_callback(self, msg):
        self.cmd_queue.put((self.priority_map['keyboard'], time.time(), msg, 'keyboard'))

    def joy_callback(self, msg):
        self.cmd_queue.put((self.priority_map['joy'], time.time(), msg, 'joy'))

    def autonomous_callback(self, msg):
        self.cmd_queue.put((self.priority_map['autonomous'], time.time(), msg, 'autonomous'))

    def publish_cmd(self):
        latest_msg = None
        seen = set()

        while not self.cmd_queue.empty():
            priority, timestamp, msg, source = self.cmd_queue.get()
            if source not in seen:
                seen.add(source)
                if latest_msg is None or priority < latest_msg[0]:
                    latest_msg = (priority, msg, source)

        if latest_msg:
            _, msg, source = latest_msg
            self.cmd_vel_pub.publish(msg)

            type_msg = String()
            type_msg.data = source
            self.cmd_type_pub.publish(type_msg)

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