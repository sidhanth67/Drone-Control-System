# This node obtains the camera image, processes it and sends back coordinates to the controller.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10
        )
        # Publishes to its own dedicated topic - does NOT collide with the
        # Gazebo plugin's ground-truth odometry topic anymore.
        self.pos_publisher = self.create_publisher(Point, '/vision/estimated_position', 10)

        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        # OpenCV 5 exposes marker detection through ArucoDetector.  Keeping
        # the detector as node state also avoids rebuilding it for every frame.
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        self.marker_length = 0.20

        self.camera_matrix = np.array([[381.4, 0.0, 320.0],
                               [0.0, 381.4, 240.0],
                               [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        self.get_logger().info("ArUco Tracker Node Initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return

        half = self.marker_length / 2.0
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        try:
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners[0][0], self.camera_matrix, self.dist_coeffs
            )
        except Exception as e:
            self.get_logger().warn(f"solvePnP failed: {e}")
            return

        if not success:
            return

        # solvePnP returns a point in the OpenCV optical camera frame:
        # x is image-right, y is image-down, and z is away from the camera.
        # The overhead camera is pitched down toward the Gazebo ground plane,
        # so its optical XY axes do not match Gazebo world XY.  Publish the
        # planar position in world coordinates so controller targets can be
        # written directly as Gazebo (x, y) points.
        pos_msg = Point()
        pos_msg.x = -float(tvec[1])  # Gazebo world X
        pos_msg.y = -float(tvec[0])  # Gazebo world Y
        pos_msg.z = float(tvec[2])

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
