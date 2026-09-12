import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

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

        # Subscribes to the ArUco node's vision-based position estimate,
        # NOT the Gazebo plugin's ground-truth odometry.
        self.subscription = self.create_subscription(
            Point,
            '/vision/estimated_position',
            self.position_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_x = 2.0
        self.target_y = 1.0
        self.target_z = 0.0
        self.current_pos = None
        self.last_position_time = None

        # The ArUco estimate is noisy by a few pixels.  Commanding velocity
        # inside this radius makes the planar model hunt instead of stop.
        self.position_tolerance = 0.05
        # A stale position must never keep the previous velocity alive.
        self.position_timeout = 0.25

        self.pid_x = PIDController(kp=3.123, ki=0.872, kd=0.0, min_output=-2.0, max_output=2.0)
        self.pid_y = PIDController(kp=3.123, ki=0.872, kd=0.0, min_output=-2.0, max_output=2.0)
        self.pid_z = PIDController(kp=1.5, ki=0.0, kd=0.2, min_output=-1.5, max_output=1.5)

        self.dt = 1.0 / 30.0
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Tuned Drone Controller Node Initialized.")

    def position_callback(self, msg):
        self.current_pos = msg
        self.last_position_time = self.get_clock().now()

    def stop_motion(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.cmd_publisher.publish(Twist())

    def control_axis(self, controller, target, current):
        if abs(target - current) <= self.position_tolerance:
            controller.reset()
            return 0.0
        return controller.compute(target, current, self.dt)

    def control_loop(self):
        if self.current_pos is None:
            return

        position_age = (self.get_clock().now() - self.last_position_time).nanoseconds / 1e9
        if position_age > self.position_timeout:
            self.stop_motion()
            return

        cmd = Twist()

        cmd.linear.x = self.control_axis(self.pid_x, self.target_x, self.current_pos.x)
        cmd.linear.y = self.control_axis(self.pid_y, self.target_y, self.current_pos.y)
        # libgazebo_ros_planar_move only implements XY motion, so publishing a
        # Z command cannot move this model and is kept at zero deliberately.
        cmd.linear.z = 0.0

        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
