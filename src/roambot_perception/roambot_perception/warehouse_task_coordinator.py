import re

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String


class WarehouseTaskCoordinator(Node):
    def __init__(self):
        super().__init__("warehouse_task_coordinator")

        report_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.confirmed_inventory = {}
        self.active_task = None
        self.retry_timer = None

        self.dispatch_pub = self.create_publisher(
            String,
            "/service/inventory/dispatch_request",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/warehouse/task_status",
            10,
        )

        self.create_subscription(
            String,
            "/scout/inventory/scan_report",
            self.scan_report_callback,
            report_qos,
        )
        self.create_subscription(
            String,
            "/warehouse/inventory_request",
            self.request_callback,
            10,
        )
        self.create_subscription(
            String,
            "/service/inventory/dispatch_status",
            self.service_status_callback,
            10,
        )

        self.get_logger().info(
            "Coordinator ready. Waiting for Scout scan report."
        )

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    def scan_report_callback(self, message):
        self.confirmed_inventory = {
            int(tag_id): shelf_name
            for shelf_name, tag_id in re.findall(
                r"(shelf_[123]):[^|]*confirmed id=(\d+)",
                message.data,
            )
        }

        self.publish_status(
            "Scout inventory available: "
            f"{self.confirmed_inventory or 'none'}"
        )

    def send_dispatch(self, shelf_name):
        dispatch_request = String()
        dispatch_request.data = shelf_name
        self.dispatch_pub.publish(dispatch_request)

    def request_callback(self, message):
        match = re.fullmatch(
            r"(?:id\s*=\s*)?(\d+)",
            message.data.strip(),
        )

        if match is None:
            self.publish_status(
                "Invalid inventory request. Send 10, 11, or 12."
            )
            return

        requested_id = int(match.group(1))

        if self.active_task is not None:
            self.publish_status(
                "A Service task is already in progress."
            )
            return

        shelf_name = self.confirmed_inventory.get(requested_id)

        if shelf_name is None:
            self.publish_status(
                f"Inventory id={requested_id} was not confirmed by Scout."
            )
            return

        self.active_task = {
            "id": requested_id,
            "shelf": shelf_name,
            "attempts": 0,
        }

        self.send_dispatch(shelf_name)

        self.publish_status(
            f"Task accepted: id={requested_id}, "
            f"dispatching Service to {shelf_name}."
        )

    def retry_dispatch(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

        if self.active_task is None:
            return

        shelf_name = self.active_task["shelf"]
        requested_id = self.active_task["id"]

        self.publish_status(
            f"Retrying Service dispatch to {shelf_name} "
            f"for inventory id={requested_id}."
        )
        self.send_dispatch(shelf_name)

    def service_status_callback(self, message):
        if self.active_task is None:
            return

        shelf_name = self.active_task["shelf"]
        requested_id = self.active_task["id"]

        if f"Service reached {shelf_name}" in message.data:
            self.publish_status(
                f"Task complete: Service reached {shelf_name} "
                f"for inventory id={requested_id}."
            )
            self.active_task = None
            return

        if f"Service could not reach {shelf_name}" not in message.data:
            return

        if self.active_task["attempts"] == 0:
            self.active_task["attempts"] = 1

            self.publish_status(
                f"First attempt failed for inventory id={requested_id}. "
                "Retrying once in 2 seconds."
            )
            self.retry_timer = self.create_timer(
                2.0,
                self.retry_dispatch,
            )
            return

        self.publish_status(
            f"Task failed after one retry: Service could not reach "
            f"{shelf_name} for inventory id={requested_id}."
        )
        self.active_task = None


def main():
    rclpy.init()
    node = WarehouseTaskCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
