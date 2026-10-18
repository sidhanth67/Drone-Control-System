import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('marker_length', 0.20)
        self.declare_parameter('camera_position', [0.0, 0.0, 4.0])
        self.declare_parameter('camera_pitch', 1.5707)
        self.declare_parameter('world_frame', 'odom')
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.pos_publisher = self.create_publisher(PointStamped, '/vision/estimated_position', 10)

        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        self.marker_length = self.get_parameter('marker_length').value
        if self.marker_length <= 0:
            raise ValueError('marker_length must be positive')
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/overhead_camera/camera_info', self.camera_info_callback,
            qos_profile_sensor_data
        )
        pitch = self.get_parameter('camera_pitch').value
        c, s = np.cos(pitch), np.sin(pitch)
        self.optical_to_world = np.array([[0, -s, c], [-1, 0, 0], [0, -c, -s]])
        self.camera_position = np.array(self.get_parameter('camera_position').value)
        self.world_frame = self.get_parameter('world_frame').value

        self.get_logger().info("ArUco Tracker Node Initialized.")

    def camera_info_callback(self, msg):
        matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if np.isfinite(matrix).all() and matrix[0, 0] > 0 and matrix[1, 1] > 0:
            self.camera_matrix = matrix
            self.dist_coeffs = np.asarray(msg.d, dtype=np.float64)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return

        matches = np.flatnonzero(ids.reshape(-1) == self.target_marker_id)
        if matches.size == 0:
            return
        marker_corners = corners[int(matches[0])][0]

        half = self.marker_length / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        try:
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners, self.camera_matrix, self.dist_coeffs
            )
        except Exception as e:
            self.get_logger().warn(f"solvePnP failed: {e}")
            return

        if not success:
            return

        translation = tvec.reshape(3)
        if not np.isfinite(translation).all() or translation[2] <= 0:
            return
        position = self.camera_position + self.optical_to_world @ translation
        pos_msg = PointStamped()
        pos_msg.header.stamp = msg.header.stamp
        pos_msg.header.frame_id = self.world_frame
        pos_msg.point.x, pos_msg.point.y, pos_msg.point.z = map(float, position)

        self.pos_publisher.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
