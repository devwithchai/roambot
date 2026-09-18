import math
import re
import time

import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

from roambot_perception.traffic_routes import (
    DOORWAYS,
    SCOUT_DESK_BAY,
)


INSPECTION_POSES = [
    {
        "name": "shelf_1",
        "expected_id": 10,
        "x": 3.833,
        "y": -0.028,
        "yaw": -0.040,
    },
    {
        "name": "shelf_2",
        "expected_id": 11,
        "x": 3.760,
        "y": -1.464,
        "yaw": 0.041,
    },
    {
        "name": "shelf_3",
        "expected_id": 12,
        "x": 3.911,
        "y": -2.990,
        "yaw": 0.033,
    },
]

DOOR_REQUEST_DISTANCE = 0.8
DOOR_STOP_DISTANCE = 0.35
DOOR_RELEASE_DISTANCE = 0.35


class InventoryScanMission(BasicNavigator):
    def __init__(self):
        super().__init__(
            node_name="inventory_scan_mission",
            namespace="scout",
        )

        self.visible_ids = set()
        self.observed_ids = set()
        self.granted_doors = set()
        self.current_map_pose = None

        report_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.report_pub = self.create_publisher(
            String,
            "inventory/scan_report",
            report_qos,
        )
        self.traffic_request_pub = self.create_publisher(
            String,
            "/warehouse/traffic/request",
            10,
        )
        self.traffic_release_pub = self.create_publisher(
            String,
            "/warehouse/traffic/release",
            10,
        )

        self.create_subscription(
            String,
            "inventory/detections",
            self.detection_callback,
            10,
        )
        self.create_subscription(
            String,
            "/warehouse/traffic/grant",
            self.traffic_grant_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self.amcl_pose_callback,
            10,
        )

    def detection_callback(self, message):
        self.visible_ids = {
            int(tag_id)
            for tag_id in re.findall(r"id=(\d+)", message.data)
        }
        self.observed_ids.update(self.visible_ids)

    def traffic_grant_callback(self, message):
        try:
            robot_name, door_name = message.data.strip().split(":", 1)
        except ValueError:
            return

        if robot_name == "scout":
            self.granted_doors.add(door_name)

    def amcl_pose_callback(self, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.current_map_pose = pose

    def make_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def wait_for_seconds(self, seconds):
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def make_transit_point(self, point, next_point):
        return {
            "x": point["x"],
            "y": point["y"],
            "yaw": math.atan2(
                next_point["y"] - point["y"],
                next_point["x"] - point["x"],
            ),
        }

    def navigate_to(self, point, label, retries=0):
        for attempt in range(retries + 1):
            attempt_label = (
                f"{label} (attempt {attempt + 1}/{retries + 1})"
            )
            self.get_logger().info(
                f"Navigating to {attempt_label}."
            )

            self.goToPose(
                self.make_goal(
                    point["x"],
                    point["y"],
                    point["yaw"],
                )
            )

            while not self.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.getResult() == TaskResult.SUCCEEDED:
                return True

            if attempt < retries:
                self.get_logger().warning(
                    f"Navigation to {label} did not complete. "
                    "Retrying once."
                )
                self.wait_for_seconds(1.0)

        return False

    def path_length(self, path):
        if path is None or len(path.poses) < 2:
            return float("inf")

        distance = 0.0
        for previous, current in zip(path.poses, path.poses[1:]):
            dx = current.pose.position.x - previous.pose.position.x
            dy = current.pose.position.y - previous.pose.position.y
            distance += math.hypot(dx, dy)

        return distance

    def route_cost(self, planner, waypoints):
        if self.current_map_pose is None:
            self.get_logger().warning("No Scout AMCL pose is available for route planning.")
            return float("inf")

        total_cost = 0.0
        previous_goal = self.current_map_pose

        for point in waypoints:
            goal = self.make_goal(
                point["x"],
                point["y"],
                point["yaw"],
            )

            try:
                path = planner.getPath(
                    previous_goal,
                    goal,
                )
            except Exception as error:
                self.get_logger().warning(f"Could not calculate route cost: {error}")
                return float("inf")

            segment_cost = self.path_length(path)
            if math.isinf(segment_cost):
                return float("inf")

            total_cost += segment_cost
            previous_goal = goal

        return total_cost

    def choose_return_route(self):
        if self.current_map_pose is None:
            self.get_logger().error(
                "Scout has no AMCL pose for return-route selection."
            )
            return None

        planner = BasicNavigator(
            node_name="scout_route_planner",
            namespace="scout",
        )

        best_route = None

        try:
            for door_name, doorway in DOORWAYS.items():
                for clearance_name in (
                    "inventory_left",
                    "inventory_right",
                ):
                    waypoints = [
                        doorway[clearance_name],
                        doorway["service_side"],
                        SCOUT_DESK_BAY,
                    ]
                    cost = self.route_cost(planner, waypoints)

                    self.get_logger().info(
                        f"Route candidate {door_name} via "
                        f"{clearance_name}: {cost:.2f} m"
                    )

                    if (
                        best_route is None
                        or cost < best_route["cost"]
                    ):
                        best_route = {
                            "door_name": door_name,
                            "clearance_name": clearance_name,
                            "cost": cost,
                        }
        finally:
            planner.destroyNode()

        return best_route

    def request_door(self, door_name):
        self.granted_doors.discard(door_name)

        request = String()
        request.data = f"scout:{door_name}"
        self.traffic_request_pub.publish(request)

        self.get_logger().info(
            f"Scout requested access to {door_name}."
        )

    def wait_for_door(self, door_name):
        deadline = time.monotonic() + 60.0
        while (
            door_name not in self.granted_doors
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if door_name not in self.granted_doors:
            self.get_logger().error(
                f"Timed out waiting for {door_name} access."
            )
            return False

        return True

    def release_door(self, door_name):
        release = String()
        release.data = f"scout:{door_name}"
        self.traffic_release_pub.publish(release)

        self.get_logger().info(
            f"Scout released {door_name}."
        )

    def distance_to(self, point):
        if self.current_map_pose is None:
            return float("inf")

        position = self.current_map_pose.pose.position
        return math.hypot(
            position.x - point["x"],
            position.y - point["y"],
        )

    def navigate_through_door(
        self,
        door_name,
        entry_point,
        exit_point,
        destination,
        destination_label,
    ):
        entry = self.make_transit_point(entry_point, exit_point)
        exit_point = self.make_transit_point(exit_point, destination)
        poses = [
            self.make_goal(entry["x"], entry["y"], entry["yaw"]),
            self.make_goal(
                exit_point["x"],
                exit_point["y"],
                exit_point["yaw"],
            ),
            self.make_goal(
                destination["x"],
                destination["y"],
                destination["yaw"],
            ),
        ]

        self.get_logger().info(
            f"Navigating through {door_name} to {destination_label}."
        )
        self.goThroughPoses(poses)

        requested = False
        granted = False
        released = False

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            entry_distance = self.distance_to(entry)

            if not requested and entry_distance <= DOOR_REQUEST_DISTANCE:
                self.request_door(door_name)
                requested = True

            if requested and not granted:
                if door_name in self.granted_doors:
                    granted = True
                    self.get_logger().info(
                        f"Scout received access to {door_name}."
                    )
                elif entry_distance <= DOOR_STOP_DISTANCE:
                    self.get_logger().info(
                        f"Scout waiting before {door_name}."
                    )
                    self.cancelTask()

                    while not self.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)

                    if not self.wait_for_door(door_name):
                        return False

                    granted = True
                    self.get_logger().info(
                        f"Scout received access to {door_name}."
                    )
                    self.goThroughPoses(poses)

            if (
                granted
                and not released
                and self.distance_to(exit_point) <= DOOR_RELEASE_DISTANCE
            ):
                self.release_door(door_name)
                released = True

        if granted and not released:
            self.release_door(door_name)

        return self.getResult() == TaskResult.SUCCEEDED

    def return_to_desk(self):
        self.get_logger().info(
            "Inspection complete. Selecting a return route to the desk."
        )

        route = self.choose_return_route()
        if route is None or math.isinf(route["cost"]):
            self.get_logger().error(
                "No feasible return route to the Scout desk bay."
            )
            return

        door_name = route["door_name"]
        clearance_name = route["clearance_name"]
        doorway = DOORWAYS[door_name]

        self.get_logger().info(
            f"Selected {door_name} via {clearance_name} "
            f"({route['cost']:.2f} m)."
        )

        if self.navigate_through_door(
            door_name,
            doorway[clearance_name],
            doorway["service_side"],
            SCOUT_DESK_BAY,
            "Scout desk bay",
        ):
            self.get_logger().info(
                "Scout returned to the service-room desk."
            )
        else:
            self.get_logger().warning(
                "Scout crossed safely but could not reach its desk bay."
            )

    def inspect_shelf(self, shelf):
        self.get_logger().info(
            f"Navigating to {shelf['name']} inspection pose."
        )

        self.observed_ids.clear()

        navigation_succeeded = self.navigate_to(
            shelf,
            shelf["name"],
            retries=1,
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
        self.get_logger().info(
            "Waiting for Scout Nav2 and AMCL..."
        )
        self.waitUntilNav2Active()

        results = []
        for shelf in INSPECTION_POSES:
            result = self.inspect_shelf(shelf)
            results.append(result)
            self.get_logger().info(result)

        report = String()
        report.data = " | ".join(results)
        self.report_pub.publish(report)
        self.get_logger().info(
            f"Scan complete: {report.data}"
        )

        self.return_to_desk()


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
