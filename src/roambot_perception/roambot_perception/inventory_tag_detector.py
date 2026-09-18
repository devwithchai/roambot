import cv2

from cv_bridge import CvBridge
from rclpy import init, shutdown
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


TAG_NAMES = {
    10: "shelf_1",
    11: "shelf_2",
    12: "shelf_3",
}


class InventoryTagDetector(Node):
    def __init__(self):
        super().__init__("inventory_tag_detector")

        self.bridge = CvBridge()
        self.last_detection = None

        self.dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.create_subscription(
            Image,
            "/scout/camera/image",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.detections_pub = self.create_publisher(
            String,
            "/scout/inventory/detections",
            10,
        )
        self.annotated_image_pub = self.create_publisher(
            Image,
            "/scout/inventory/annotated_image",
            10,
        )

        self.get_logger().info(
            "Inventory tag detector is listening on /scout/camera/image"
        )

    def image_callback(self, image_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

        annotated = image.copy()
        detected_tags = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            for tag_id in ids.flatten():
                tag_id = int(tag_id)
                if tag_id in TAG_NAMES:
                    detected_tags.append(
                        f"id={tag_id}, location={TAG_NAMES[tag_id]}"
                    )

        detected_tags.sort()
        detection_text = "; ".join(detected_tags) if detected_tags else "none"

        message = String()
        message.data = detection_text
        self.detections_pub.publish(message)

        if detection_text != self.last_detection:
            self.get_logger().info(f"Inventory tags detected: {detection_text}")
            self.last_detection = detection_text

        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated,
            encoding="bgr8",
        )
        annotated_msg.header = image_msg.header
        self.annotated_image_pub.publish(annotated_msg)


def main():
    init()
    node = InventoryTagDetector()

    try:
        node.get_logger().info("Inventory tag detector started")
        from rclpy import spin
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()


if __name__ == "__main__":
    main()