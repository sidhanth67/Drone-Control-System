from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, Twist

from drone_control_pkg.aruco_tracker_node import ArucoTrackerNode
from drone_control_pkg.command_watchdog_node import CommandWatchdogNode
from drone_control_pkg.controller_node import DroneControllerNode


def test_watchdog_stops_after_command_loss():
    node = SimpleNamespace(
        last_command_time=10.0, timeout=0.25, publisher=Mock()
    )
    with patch('drone_control_pkg.command_watchdog_node.time.monotonic', return_value=10.3):
        CommandWatchdogNode.check_timeout(node)
    assert node.publisher.publish.call_args.args[0] == Twist()


def test_wrong_goal_frame_is_rejected():
    node = SimpleNamespace(world_frame='odom', get_logger=Mock())
    goal = PoseStamped()
    goal.header.frame_id = 'camera_link'
    DroneControllerNode.goal_callback(node, goal)
    assert not hasattr(node, 'target_x')


def test_wrong_position_frame_is_rejected():
    node = SimpleNamespace(world_frame='odom', current_pos=None)
    position = PointStamped()
    position.header.frame_id = 'camera_link'
    DroneControllerNode.position_callback(node, position)
    assert node.current_pos is None


def test_tracker_selects_target_and_transforms_position():
    matrix = np.array([[381.4, 0, 320], [0, 381.4, 240], [0, 0, 1.]])
    objects = np.array([[-.1, .1, 0], [.1, .1, 0], [.1, -.1, 0], [-.1, -.1, 0]])
    rotation = np.array([np.pi, 0., 0.])
    translation = np.array([-1., -2., 3.8])
    corners, _ = cv2.projectPoints(objects, rotation, translation, matrix, np.zeros(4))
    node = SimpleNamespace(
        camera_matrix=matrix, dist_coeffs=np.zeros(4), target_marker_id=0,
        marker_length=.2, camera_position=np.array([0., 0., 4.]),
        optical_to_world=np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]),
        world_frame='odom', bridge=Mock(), detector=Mock(), pos_publisher=Mock()
    )
    node.bridge.imgmsg_to_cv2.return_value = np.zeros((480, 640, 3), np.uint8)
    node.detector.detectMarkers.return_value = (
        [np.zeros((1, 4, 2)), corners.reshape(1, 4, 2).astype(np.float32)],
        np.array([[7], [0]]), None
    )
    msg = PointStamped()
    ArucoTrackerNode.image_callback(node, msg)
    result = node.pos_publisher.publish.call_args.args[0]
    np.testing.assert_allclose([result.point.x, result.point.y, result.point.z],
                               [2., 1., .2], atol=1e-4)
    assert result.header.frame_id == 'odom'
