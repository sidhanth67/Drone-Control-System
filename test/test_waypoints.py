from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point, PoseStamped
from rclpy.time import Time

from drone_control_pkg.controller_node import DroneControllerNode, PIDController


def make_controller():
    node = SimpleNamespace(
        has_goal=True, following_waypoints=True, setpoint_index=0,
        setpoints=[(0.0, 0.0), (1.0, 1.0)], target_x=0.0, target_y=0.0,
        current_pos=Point(), last_position_time=Time(seconds=1),
        position_timeout=0.25, position_tolerance=0.05,
        world_frame='odom', goal_bounds=(-2.0, 2.0, -2.7, 2.7),
        pid_x=PIDController(1, 0, 0), pid_y=PIDController(1, 0, 0),
        dt=1 / 30, cmd_publisher=Mock(), get_logger=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1))
    )
    for method in ('constrain_goal', 'stop_motion', 'control_axis'):
        setattr(node, method, getattr(DroneControllerNode, method).__get__(node))
    return node


def test_arrival_advances_once_and_stops_before_next_tick():
    node = make_controller()
    DroneControllerNode.control_loop(node)
    assert node.setpoint_index == 1
    assert (node.target_x, node.target_y) == (1.0, 1.0)
    assert node.cmd_publisher.publish.call_count == 1
    assert node.cmd_publisher.publish.call_args.args[0].linear.x == 0
    DroneControllerNode.control_loop(node)
    assert node.setpoint_index == 1
    assert node.cmd_publisher.publish.call_args.args[0].linear.x > 0


def test_final_waypoint_completes_mission():
    node = make_controller()
    node.setpoint_index = 1
    node.target_x = node.target_y = 1.0
    node.current_pos.x = node.current_pos.y = 1.0
    DroneControllerNode.control_loop(node)
    assert not node.has_goal
    DroneControllerNode.control_loop(node)
    assert node.cmd_publisher.publish.call_count == 1


def test_manual_goal_interrupts_route_and_completes_independently():
    node = make_controller()
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 0.5
    DroneControllerNode.goal_callback(node, goal)
    DroneControllerNode.control_loop(node)
    assert not node.following_waypoints
    assert node.target_x == 0.5
    assert node.setpoint_index == 0
    node.current_pos.x = 0.5
    DroneControllerNode.control_loop(node)
    assert not node.has_goal
    assert node.setpoint_index == 0


def test_stale_tracking_does_not_advance_route():
    node = make_controller()
    node.last_position_time = Time()
    DroneControllerNode.control_loop(node)
    assert node.setpoint_index == 0
    assert node.cmd_publisher.publish.call_args.args[0].linear.x == 0


def test_next_waypoint_is_constrained():
    node = make_controller()
    node.setpoints[1] = (8.0, -8.0)
    DroneControllerNode.control_loop(node)
    assert (node.target_x, node.target_y) == (2.0, -2.7)
