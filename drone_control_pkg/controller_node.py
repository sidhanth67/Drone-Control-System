import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist

class PIDController:
    def __init__(self, kp, ki, kd, min_output=-2.0, max_output=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, current_value, dt):
        error = setpoint - current_value

        p_term = self.kp * error

        derivative = (error - self.prev_error) / dt if self.kd > 0 else 0.0
        d_term = self.kd * derivative
        self.prev_error = error

        potential_integral = self.integral + (self.ki * error * dt)
        output_unconstrained = p_term + potential_integral + d_term

        if output_unconstrained > self.max_output and error > 0:
            pass
        elif output_unconstrained < self.min_output and error < 0:
            pass
        else:
            self.integral = potential_integral

        output = p_term + self.integral + d_term

        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output

        return output

class DroneControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 1.0)
        self.declare_parameter('wait_for_goal', False)
        self.has_goal = not self.get_parameter('wait_for_goal').value

        self.subscription = self.create_subscription(
            PointStamped,
            '/vision/estimated_position',
            self.position_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.declare_parameter('world_frame', 'odom')
        self.world_frame = self.get_parameter('world_frame').value
        self.current_pos = None
        self.last_position_time = None

        self.position_tolerance = 0.05
        self.position_timeout = 0.25

        self.pid_x = PIDController(kp=3.123, ki=0.872, kd=0.0, min_output=-2.0, max_output=2.0)
        self.pid_y = PIDController(kp=3.123, ki=0.872, kd=0.0, min_output=-2.0, max_output=2.0)

        self.dt = 1.0 / 30.0
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f"Tuned Drone Controller Node Initialized. "
            f"Target: ({self.target_x:.2f}, {self.target_y:.2f})"
        )

    def position_callback(self, msg):
        if msg.header.frame_id != self.world_frame:
            return
        if not all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z)):
            return
        self.current_pos = msg.point
        self.last_position_time = rclpy.time.Time.from_msg(msg.header.stamp)

    def goal_callback(self, msg):
        if msg.header.frame_id != self.world_frame:
            self.get_logger().warn('Goal frame must match ' + self.world_frame)
            return
        if not all(math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y)):
            return
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.pid_x.reset()
        self.pid_y.reset()

        self.has_goal = True

    def stop_motion(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.cmd_publisher.publish(Twist())

    def control_axis(self, controller, target, current):
        if abs(target - current) <= self.position_tolerance:
            controller.reset()
            return 0.0
        return controller.compute(target, current, self.dt)

    def control_loop(self):
        if not self.has_goal:
            return
        if self.current_pos is None:
            return

        position_age = (self.get_clock().now() - self.last_position_time).nanoseconds / 1e9
        if position_age < 0 or position_age > self.position_timeout:
            self.stop_motion()
            return

        cmd = Twist()

        cmd.linear.x = self.control_axis(self.pid_x, self.target_x, self.current_pos.x)
        cmd.linear.y = self.control_axis(self.pid_y, self.target_y, self.current_pos.y)

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
