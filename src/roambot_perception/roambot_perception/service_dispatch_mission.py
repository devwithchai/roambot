import json
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
    SERVICE_DESK_BAY,
)


SHELF_POSES = {
    "shelf_1": {
        "name": "shelf_1",
        "id": 10,
        "x": 3.833,
        "y": -0.028,
        "yaw": -0.040,
    },
    "shelf_2": {
        "name": "shelf_2",
        "id": 11,
        "x": 3.760,
        "y": -1.464,
        "yaw": 0.041,
    },
    "shelf_3": {
        "name": "shelf_3",
        "id": 12,
        "x": 3.911,
        "y": -2.990,
        "yaw": 0.033,
    },
}

DOOR_REQUEST_DISTANCE = 0.8
DOOR_STOP_DISTANCE = 0.35
DOOR_RELEASE_DISTANCE = 0.35


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
        self.granted_doors = set()
        self.current_map_pose = None

        self.status_pub = self.create_publisher(
            String,
            "inventory/dispatch_status",
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            "/warehouse/service_state",
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
            report_qos,
        )

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    def publish_state(
        self,
        state,
        current_shelf=None,
        message=None,
    ):
        status = String()
        status.data = json.dumps(
            {
                "state": state,
                "current_shelf": current_shelf,
                "message": message or "",
            }
        )
        self.state_pub.publish(status)

    def scan_report_callback(self, message):
        self.confirmed_shelves = {
            shelf_name: int(tag_id)
            for shelf_name, tag_id in re.findall(
                r"(shelf_[123]):[^|]*confirmed id=(\d+)",
                message.data,
            )
        }

        self.get_logger().info(
            "Scout scan report received: "
            f"{self.confirmed_shelves or 'no confirmed shelves'}"
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
                f"{shelf_name} was not confirmed by Scout; "
                "dispatch skipped."
            )
            return

        self.pending_shelf = shelf_name
        self.publish_state(
            "queued",
            current_shelf=shelf_name,
            message=f"Service dispatch accepted for {shelf_name}.",
        )
        self.get_logger().info(
            f"Dispatch accepted for {shelf_name}."
        )

    def traffic_grant_callback(self, message):
        try:
            robot_name, door_name = message.data.strip().split(":", 1)
        except ValueError:
            return

        if robot_name == "service":
            self.granted_doors.add(door_name)

    def amcl_pose_callback(self, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.current_map_pose = pose

    def wait_for_map_pose(self, timeout_sec=15.0):
        self.get_logger().info(
            "Waiting for Service AMCL pose..."
        )

        deadline = time.monotonic() + timeout_sec
        while (
            self.current_map_pose is None
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_map_pose is None:
            self.get_logger().error(
                "Service AMCL pose was not received."
            )
            return False

        self.get_logger().info(
            "Service AMCL pose is ready."
        )
        return True

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
            self.get_logger().info(
                f"Navigating to {label} "
                f"(attempt {attempt + 1}/{retries + 1})."
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
            self.get_logger().warning(
                "No Service AMCL pose is available for route planning."
            )
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
                self.get_logger().warning(
                    f"Could not calculate route cost: {error}"
                )
                return float("inf")

            segment_cost = self.path_length(path)
            if math.isinf(segment_cost):
                return float("inf")

            total_cost += segment_cost
            previous_goal = goal

        return total_cost

    def choose_outbound_route(self, shelf):
        if self.current_map_pose is None:
            self.get_logger().error(
                "Service has no AMCL pose for route selection."
            )
            return None

        planner = BasicNavigator(
            node_name="service_route_planner",
            namespace="service",
        )
        best_route = None

        try:
            for door_name, doorway in DOORWAYS.items():
                for clearance_name in (
                    "inventory_left",
                    "inventory_right",
                ):
                    waypoints = [
                        doorway["service_side"],
                        doorway[clearance_name],
                        shelf,
                    ]
                    cost = self.route_cost(planner, waypoints)

                    self.get_logger().info(
                        f"Outbound candidate {door_name} via "
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

    def choose_return_route(self):
        if self.current_map_pose is None:
            self.get_logger().error(
                "Service has no AMCL pose for return-route selection."
            )
            return None

        planner = BasicNavigator(
            node_name="service_return_planner",
            namespace="service",
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
                        SERVICE_DESK_BAY,
                    ]
                    cost = self.route_cost(planner, waypoints)

                    self.get_logger().info(
                        f"Return candidate {door_name} via "
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
        request.data = f"service:{door_name}"
        self.traffic_request_pub.publish(request)

        self.get_logger().info(
            f"Service requested access to {door_name}."
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
        release.data = f"service:{door_name}"
        self.traffic_release_pub.publish(release)

        self.get_logger().info(
            f"Service released {door_name}."
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
                        f"Service received access to {door_name}."
                    )
                elif entry_distance <= DOOR_STOP_DISTANCE:
                    self.get_logger().info(
                        f"Service waiting before {door_name}."
                    )
                    self.cancelTask()

                    while not self.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)

                    if not self.wait_for_door(door_name):
                        return False

                    granted = True
                    self.get_logger().info(
                        f"Service received access to {door_name}."
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

    def travel_to_shelf(self, shelf):
        route = self.choose_outbound_route(shelf)
        if route is None or math.isinf(route["cost"]):
            self.get_logger().error(
                "No feasible Service route to the requested shelf."
            )
            return False

        door_name = route["door_name"]
        clearance_name = route["clearance_name"]
        doorway = DOORWAYS[door_name]

        self.get_logger().info(
            f"Selected outbound {door_name} via {clearance_name} "
            f"({route['cost']:.2f} m)."
        )

        return self.navigate_through_door(
            door_name,
            doorway["service_side"],
            doorway[clearance_name],
            shelf,
            shelf["name"],
        )

    def return_to_desk(self):
        self.publish_state(
            "returning",
            message="Service is returning to its desk.",
        )
        self.get_logger().info(
            "Service task complete. Selecting a return route to the desk."
        )

        route = self.choose_return_route()
        if route is None or math.isinf(route["cost"]):
            self.get_logger().error(
                "No feasible Service return route to the desk."
            )
            return

        door_name = route["door_name"]
        clearance_name = route["clearance_name"]
        doorway = DOORWAYS[door_name]

        self.get_logger().info(
            f"Selected return {door_name} via {clearance_name} "
            f"({route['cost']:.2f} m)."
        )

        if self.navigate_through_door(
            door_name,
            doorway[clearance_name],
            doorway["service_side"],
            SERVICE_DESK_BAY,
            "Service desk bay",
        ):
            self.publish_status(
                "Service returned to the service-room desk."
            )
            self.publish_state(
                "idle",
                message="Service is parked at its desk.",
            )
        else:
            self.get_logger().warning(
                "Service crossed safely but could not reach "
                "its desk bay."
            )

    def dispatch_to_shelf(self, shelf_name):
        shelf = SHELF_POSES[shelf_name]

        self.publish_state(
            "travelling_to_shelf",
            current_shelf=shelf_name,
            message=f"Service is travelling to {shelf_name}.",
        )
        self.get_logger().info(
            f"Service navigating to {shelf_name}."
        )

        if self.travel_to_shelf(shelf):
            self.publish_state(
                "at_shelf",
                current_shelf=shelf_name,
                message=f"Service reached {shelf_name}.",
            )
            self.publish_status(
                f"Service reached {shelf_name} "
                f"for inventory tag id={shelf['id']}"
            )
            self.return_to_desk()
        else:
            self.publish_state(
                "failed",
                current_shelf=shelf_name,
                message=f"Service could not reach {shelf_name}.",
            )
            self.publish_status(
                f"Service could not reach {shelf_name}"
            )

    def run(self):
        self.get_logger().info(
            "Waiting for Service Nav2 and Scout scan report..."
        )
        self.waitUntilNav2Active()

        if not self.wait_for_map_pose():
            return

        self.publish_state(
            "idle",
            message="Service is ready for inventory dispatch.",
        )
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
