import re
import time
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)
from std_msgs.msg import String


INSPECTION_POSES = [
    {"name": "shelf_1", "expected_id": 10, "x": 3.833, "y": -0.028, "yaw": -0.040},
    {"name": "shelf_2", "expected_id": 11, "x": 3.760, "y": -1.464, "yaw": 0.041},
    {"name": "shelf_3", "expected_id": 12, "x": 3.911, "y": -2.99, "yaw": 0.033},
]

class InventoryScanMission(BasicNavigator):
    def __init__(self):
        super().__init__(
            node_name="inventory_scan_mission",
            namespace="scout",
        )

        self.visible_ids = set()
        self.observed_ids = set()
        self.report_pub = self.create_publisher(
            String,
            "inventory/scan_report",
            10,
        )
        self.create_subscription(
            String,
            "inventory/detections",
            self.detection_callback,
            10,
        )

    def detection_callback(self, message):
        self.visible_ids = {
            int(tag_id)
            for tag_id in re.findall(r"id=(\d+)", message.data)
        }
        self.observed_ids.update(self.visible_ids)

    def make_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def inspect_shelf(self, shelf):
        self.get_logger().info(f"Navigating to {shelf['name']} inspection pose")

        self.observed_ids.clear()

        self.goToPose(
            self.make_goal(
                shelf["x"],
                shelf["y"],
                shelf["yaw"],
            )
        )

        while not self.isTaskComplete():
            pass

        navigation_succeeded = (
            self.getResult() == TaskResult.SUCCEEDED
        )

        scan_end = time.monotonic() + 4.0
        while time.monotonic() < scan_end:
            rclpy.spin_once(self, timeout_sec=0.1)

        found_ids = sorted(self.observed_ids)
        expected_id = shelf["expected_id"]

        nav_state = (
            "reached"
            if navigation_succeeded
            else "navigation incomplete"
        )

        if expected_id in found_ids:
            return (
                f"{shelf['name']}: {nav_state}, "
                f"confirmed id={expected_id}"
            )

        if found_ids:
            return (
                f"{shelf['name']}: {nav_state}, "
                f"saw ids={found_ids}, expected id={expected_id}"
            )

        return f"{shelf['name']}: {nav_state}, no tag detected"

    def run_scan(self):
        self.get_logger().info("Waiting for Scout Nav2 and AMCL...")
        self.waitUntilNav2Active()

        results = []
        for shelf in INSPECTION_POSES:
            result = self.inspect_shelf(shelf)
            results.append(result)
            self.get_logger().info(result)

        report = String()
        report.data = " | ".join(results)
        self.report_pub.publish(report)
        self.get_logger().info(f"Scan complete: {report.data}")


def main():
    rclpy.init()
    mission = InventoryScanMission()

    try:
        mission.run_scan()
    except KeyboardInterrupt:
        pass
    finally:
        mission.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()