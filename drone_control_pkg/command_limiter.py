import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class CommandWatchdogNode(Node):
    def __init__(self):
        super().__init__('command_watchdog_node')
        self.declare_parameter('command_timeout', 0.25)
        self.timeout = self.get_parameter('command_timeout').value
        if self.timeout <= 0:
            raise ValueError('command_timeout must be positive')
        self.last_command_time = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            Twist, '/control/cmd_vel', self.command_callback, 1
        )
        self.timer = self.create_timer(
            0.05, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def command_callback(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.last_command_time = None
            self.publisher.publish(Twist())
            return
        self.last_command_time = time.monotonic()
        self.publisher.publish(msg)

    def check_timeout(self):
        if (self.last_command_time is None or
                time.monotonic() - self.last_command_time > self.timeout):
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CommandWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
