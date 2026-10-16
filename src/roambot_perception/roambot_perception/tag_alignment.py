import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class TagAlignment(Node):
    def __init__(self):
        super().__init__('tag_alignment')

        self.declare_parameter('robot', 'scout')
        self.declare_parameter('expected_tag_id', 10)
        self.declare_parameter('center_tolerance_px', 30.0)
        self.declare_parameter('required_centered_frames', 5)
        self.declare_parameter('max_angular_speed', 0.30)
        self.declare_parameter('turn_gain', 0.50)
        self.declare_parameter('search_angular_speed', 0.20)
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('turn_direction_sign', -1.0)

        self.robot = self.get_parameter('robot').value
        self.expected_tag_id = int(
            self.get_parameter('expected_tag_id').value
        )
        self.center_tolerance_px = float(
            self.get_parameter('center_tolerance_px').value
        )
        self.required_centered_frames = int(
            self.get_parameter('required_centered_frames').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.turn_gain = float(
            self.get_parameter('turn_gain').value
        )
        self.search_angular_speed = float(
            self.get_parameter('search_angular_speed').value
        )
        self.timeout_sec = float(
            self.get_parameter('timeout_sec').value
        )
        self.turn_direction_sign = float(
            self.get_parameter('turn_direction_sign').value
        )

        self.bridge = CvBridge()
        self.centered_frames = 0
        self.finished = False
        self.success = False
        self.start_time = time.monotonic()
        self.last_state = ''

        aruco = cv2.aruco
        if hasattr(aruco, 'getPredefinedDictionary'):
            self.dictionary = aruco.getPredefinedDictionary(
                aruco.DICT_4X4_50
            )
        else:
            self.dictionary = aruco.Dictionary_get(
                aruco.DICT_4X4_50
            )

        self.parameters = aruco.DetectorParameters_create()

        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{self.robot}/cmd_vel_nav',
            10,
        )

        self.create_subscription(
            Image,
            f'/{self.robot}/camera/image',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info(
            f'Aligning {self.robot} with ArUco tag '
            f'{self.expected_tag_id}.'
        )

    def publish_angular_velocity(self, angular_z):
        command = Twist()
        command.angular.z = float(angular_z)
        self.cmd_pub.publish(command)

    def stop_robot(self):
        self.publish_angular_velocity(0.0)

    def report_state(self, state):
        if state != self.last_state:
            self.get_logger().info(state)
            self.last_state = state

    def detect_markers(self, gray):
        aruco = cv2.aruco

        if hasattr(aruco, 'ArucoDetector'):
            detector = aruco.ArucoDetector(
                self.dictionary,
                self.parameters,
            )
            return detector.detectMarkers(gray)

        return aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

    def image_callback(self, image_msg):
        if self.finished:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8',
            )
        except Exception as error:
            self.get_logger().error(
                f'Camera image conversion failed: {error}'
            )
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_markers(gray)

        target_corners = None

        if ids is not None:
            for marker_corners, marker_id in zip(
                corners,
                ids.flatten(),
            ):
                if int(marker_id) == self.expected_tag_id:
                    target_corners = marker_corners
                    break

        if target_corners is None:
            self.centered_frames = 0
            self.report_state(
                f'Tag {self.expected_tag_id} not visible; searching.'
            )
            self.publish_angular_velocity(self.search_angular_speed)
            return

        image_center_x = image.shape[1] / 2.0
        tag_center_x = float(target_corners[0][:, 0].mean())
        error_px = tag_center_x - image_center_x

        if abs(error_px) <= self.center_tolerance_px:
            self.centered_frames += 1
            self.stop_robot()
            self.report_state(
                f'Tag {self.expected_tag_id} centred: '
                f'error={error_px:.1f}px '
                f'({self.centered_frames}/'
                f'{self.required_centered_frames}).'
            )

            if self.centered_frames >= self.required_centered_frames:
                self.success = True
                self.finished = True
                self.stop_robot()
                self.get_logger().info(
                    f'Alignment complete for tag '
                    f'{self.expected_tag_id}.'
                )
            return

        self.centered_frames = 0

        speed = self.turn_gain * abs(error_px) / image_center_x
        speed = max(0.08, min(self.max_angular_speed, speed))

        angular_z = (
            self.turn_direction_sign
            * math.copysign(speed, error_px)
        )

        direction = 'right' if angular_z < 0.0 else 'left'
        self.report_state(
            f'Tag {self.expected_tag_id}: '
            f'error={error_px:.1f}px; turning {direction}.'
        )
        self.publish_angular_velocity(angular_z)

    def check_timeout(self):
        if self.finished:
            return

        if time.monotonic() - self.start_time >= self.timeout_sec:
            self.stop_robot()
            self.finished = True
            self.success = False
            self.get_logger().error(
                f'Timed out while aligning tag '
                f'{self.expected_tag_id}.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TagAlignment()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

    return 0 if node.success else 1


if __name__ == '__main__':
    main()