import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute

class TeleportAndCircleNode(Node):
    def __init__(self):
        super().__init__('teleport_and_circle_node')
        
        # Publisher for velocity
        self.cmd_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Service client for teleporting
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')#args /turtle1/teleport_absolute is the service name. The service type is TeleportAbsolute.   
        # Circle parameters (Centered at 5,5 with radius 2.0)
        self.center_x = 5.0
        self.center_y = 5.0
        self.radius = 2.0
        self.linear_speed = 2.0
        self.angular_speed = self.linear_speed / self.radius
        
        # 1. Instantly teleport to (7.5) facing up
        self.teleport_turtle(7.0, 5.0, math.pi / 2.0)
        
        # 2. Start drawing the circle immediately
        self.timer = self.create_timer(0.1, self.control_loop)

    def teleport_turtle(self, x, y, theta):
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        
        # Call the service synchronously upon startup
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Teleported turtle to ({x}, {y})!')

    def control_loop(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleportAndCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()