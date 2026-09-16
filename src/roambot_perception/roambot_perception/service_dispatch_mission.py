import math
import re

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String


SHELF_POSES = {
    "shelf_1": {"id": 10, "x": 3.833, "y": -0.028, "yaw": -0.040},
    "shelf_2": {"id": 11, "x": 3.760, "y": -1.464, "yaw": 0.041},
    "shelf_3": {"id": 12, "x": 3.911, "y": -2.990, "yaw": 0.033},
}


class ServiceDispatchMission(BasicNavigator):
    def __init__(self):
        super().__init__(
            node_name="service_dispatch_mission",
            namespace="service",
        )

        report_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.confirmed_shelves = {}
        self.pending_shelf = None

        self.status_pub = self.create_publisher(
            String,
            "inventory/dispatch_status",
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
            "inventory/dispatch_request",
            self.request_callback,
            10,
        )

    def scan_report_callback(self, message):
        confirmed = {
            shelf_name: int(tag_id)
            for shelf_name, tag_id in re.findall(
                r"(shelf_[123]):[^|]*confirmed id=(\d+)",
                message.data,
            )
        }

        self.confirmed_shelves = confirmed
        self.get_logger().info(
            f"Scout scan report received: {confirmed or 'no confirmed shelves'}"
        )

    def request_callback(self, message):
        shelf_name = message.data.strip()

        if shelf_name not in SHELF_POSES:
            self.get_logger().warning(
                "Invalid request. Use shelf_1, shelf_2, or shelf_3."
            )
            return

        if shelf_name not in self.confirmed_shelves:
            self.get_logger().warning(
                f"{shelf_name} was not confirmed by Scout; dispatch skipped."
            )
            return

        self.pending_shelf = shelf_name
        self.get_logger().info(
            f"Dispatch accepted for {shelf_name}."
        )

    def make_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def dispatch_to_shelf(self, shelf_name):
        shelf = SHELF_POSES[shelf_name]

        self.get_logger().info(
            f"Service navigating to {shelf_name}."
        )

        self.goToPose(
            self.make_goal(
                shelf["x"],
                shelf["y"],
                shelf["yaw"],
            )
        )

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.getResult()
        status = String()

        if result == TaskResult.SUCCEEDED:
            status.data = (
                f"Service reached {shelf_name} "
                f"for inventory tag id={shelf['id']}"
            )
        else:
            status.data = (
                f"Service could not reach {shelf_name}"
            )

        self.status_pub.publish(status)
        self.get_logger().info(status.data)

    def run(self):
        self.get_logger().info(
            "Waiting for Service Nav2 and Scout scan report..."
        )
        self.waitUntilNav2Active()

        self.get_logger().info(
            "Ready. Send shelf_1, shelf_2, or shelf_3 "
            "to /service/inventory/dispatch_request."
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.pending_shelf is not None:
                shelf_name = self.pending_shelf
                self.pending_shelf = None
                self.dispatch_to_shelf(shelf_name)


def main():
    rclpy.init()
    mission = ServiceDispatchMission()

    try:
        mission.run()
    except KeyboardInterrupt:
        pass
    finally:
        mission.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()