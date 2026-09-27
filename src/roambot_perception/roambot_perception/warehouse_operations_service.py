import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

from roambot_perception.warehouse_operations_routes import (
    DOORWAYS,
    PACKOUT_BAYS,
    SERVICE_CHARGE_BAY,
    SHELF_POSES,
)


REQUEST_DISTANCE = 0.8
STOP_DISTANCE = 0.35
RELEASE_DISTANCE = 0.35


class WarehouseOperationsService(BasicNavigator):
    def __init__(self):
        super().__init__(
            node_name="warehouse_operations_service",
            namespace="service",
        )

        pose_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.current_pose = None
        self.granted_doors = set()
        self.pending_task = None
        self.busy = False

        self.status_pub = self.create_publisher(
            String,
            "/warehouse/operations/service_status",
            pose_qos,
        )

        self.request_pub = self.create_publisher(
            String, "/warehouse/traffic/request", 10
        )
        self.release_pub = self.create_publisher(
            String, "/warehouse/traffic/release", 10
        )

        self.create_subscription(
            String,
            "/warehouse/traffic/grant",
            self.grant_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self.pose_callback,
            pose_qos,
        )
        self.create_subscription(
            String,
            "/warehouse/operations/service_request",
            self.task_callback,
            10,
        )

    def pose_callback(self, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.current_pose = pose

    def grant_callback(self, message):
        try:
            robot, door = message.data.split(":", 1)
        except ValueError:
            return

        if robot == "service":
            self.granted_doors.add(door)

    def publish_state(self, state, message):
        status = String()
        status.data = json.dumps(
            {
                "robot": "service",
                "state": state,
                "message": message,
            }
        )
        self.status_pub.publish(status)
        self.get_logger().info(message)

    def task_callback(self, message):
        parts = message.data.strip().split(":")

        if len(parts) not in (1, 2):
            self.get_logger().warning(
                "Service request must be shelf_1:packout_a."
            )
            return

        shelf_name = parts[0]
        bay_name = parts[1] if len(parts) == 2 else "packout_a"

        if shelf_name not in SHELF_POSES:
            self.get_logger().warning("Unknown service shelf.")
            return

        if bay_name not in PACKOUT_BAYS:
            self.get_logger().warning("Unknown Packout bay.")
            return

        if self.busy or self.pending_task is not None:
            self.get_logger().warning("Service is already busy.")
            return

        self.pending_task = (shelf_name, bay_name)
        self.publish_state(
            "queued",
            f"Service queued for {shelf_name}.",
        )

    def make_goal(self, point):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = point["x"]
        goal.pose.position.y = point["y"]
        goal.pose.orientation.z = math.sin(point["yaw"] / 2.0)
        goal.pose.orientation.w = math.cos(point["yaw"] / 2.0)
        return goal

    def wait_for_pose(self):
        deadline = time.monotonic() + 15.0

        while self.current_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.current_pose is not None

    def distance_to(self, point):
        if self.current_pose is None:
            return float("inf")

        position = self.current_pose.pose.position
        return math.hypot(
            position.x - point["x"],
            position.y - point["y"],
        )

    def request_door(self, door_name):
        self.granted_doors.discard(door_name)

        message = String()
        message.data = f"service:{door_name}"
        self.request_pub.publish(message)

        self.get_logger().info(
            f"Service requested {door_name}."
        )

    def release_door(self, door_name):
        message = String()
        message.data = f"service:{door_name}"
        self.release_pub.publish(message)

        self.get_logger().info(
            f"Service released {door_name}."
        )

    def wait_for_grant(self, door_name):
        deadline = time.monotonic() + 60.0

        while (
            door_name not in self.granted_doors
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        return door_name in self.granted_doors

    def door_points(self, door_name, to_inventory):
        doorway = DOORWAYS[door_name]

        if to_inventory:
            entry = doorway["operations_side"]
            exit_point = doorway["inventory_side"]
        else:
            entry = doorway["inventory_side"]
            exit_point = doorway["operations_side"]

        # Keep the same heading from before the doorway to after it.
        crossing_yaw = math.atan2(
            exit_point["y"] - entry["y"],
            exit_point["x"] - entry["x"],
        )

        entry = {**entry, "yaw": crossing_yaw}
        exit_point = {**exit_point, "yaw": crossing_yaw}
        return entry, exit_point

    def path_length(self, path):
        if path is None or len(path.poses) < 2:
            return float("inf")

        return sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            for previous, current in zip(path.poses, path.poses[1:])
        )

    def route_cost(self, planner, points):
        previous = self.current_pose
        total = 0.0

        for point in points:
            path = planner.getPath(previous, self.make_goal(point))
            length = self.path_length(path)

            if math.isinf(length):
                return float("inf")

            total += length
            previous = self.make_goal(point)

        return total

    def choose_door(self, destination, to_inventory):
        planner = BasicNavigator(
            node_name="service_operations_planner",
            namespace="service",
        )
        best = None

        try:
            for door_name in DOORWAYS:
                entry, exit_point = self.door_points(
                    door_name,
                    to_inventory,
                )
                cost = self.route_cost(
                    planner,
                    [entry, exit_point, destination],
                )

                self.get_logger().info(
                    f"{door_name}: {cost:.2f} m"
                )

                if best is None or cost < best["cost"]:
                    best = {
                        "name": door_name,
                        "cost": cost,
                        "entry": entry,
                        "exit": exit_point,
                    }
        finally:
            planner.destroyNode()

        return best

    def navigate_route(self, points, labels, door_name=None):
        pending = list(zip(points, labels))
        retries = {label: 0 for label in labels}

        requested = False
        released = False
        entry = points[0] if door_name else None
        exit_point = points[1] if door_name else None

        while pending:
            goals = [
                self.make_goal(point)
                for point, _ in pending
            ]
            self.goThroughPoses(goals)

            poses_left = len(pending)
            waiting_for_door = False

            while not self.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

                feedback = self.getFeedback()
                if feedback is not None:
                    poses_left = max(
                        1,
                        min(
                            len(pending),
                            feedback.number_of_poses_remaining,
                        ),
                    )

                if door_name is None:
                    continue

                if (
                    not requested
                    and self.distance_to(entry) <= REQUEST_DISTANCE
                ):
                    self.request_door(door_name)
                    requested = True

                if (
                    requested
                    and door_name not in self.granted_doors
                    and self.distance_to(entry) <= STOP_DISTANCE
                ):
                    self.get_logger().info(
                        f"Waiting before {door_name}."
                    )
                    self.cancelTask()

                    while not self.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)

                    if not self.wait_for_grant(door_name):
                        self.get_logger().error(
                            f"Timed out waiting for {door_name}."
                        )
                        return False

                    waiting_for_door = True
                    break

                if (
                    requested
                    and not released
                    and door_name in self.granted_doors
                    and self.distance_to(exit_point)
                    <= RELEASE_DISTANCE
                ):
                    self.release_door(door_name)
                    released = True

            if waiting_for_door:
                continue

            if self.getResult() == TaskResult.SUCCEEDED:
                if requested and not released:
                    self.release_door(door_name)
                return True

            completed = len(pending) - poses_left

            if released:
                completed = max(completed, 2)

            pending = pending[completed:]

            if not pending:
                return False

            point, label = pending[0]
            retries[label] += 1

            if retries[label] > 1:
                self.get_logger().error(
                    f"{label} failed twice."
                )
                return False

            self.get_logger().warning(
                f"{label} failed. Retrying once."
            )
            time.sleep(1.0)

        return True

    def navigate_through_door(self, route, destination, label):
        return self.navigate_route(
            [
                route["entry"],
                route["exit"],
                destination,
            ],
            [
                f"{route['name']} entry",
                f"{route['name']} exit",
                label,
            ],
            route["name"],
        )

    def navigate_to(self, destination, label):
        return self.navigate_route(
            [destination],
            [label],
        )

    def execute_task(self, shelf_name, bay_name):
        shelf = SHELF_POSES[shelf_name]
        packout = PACKOUT_BAYS[bay_name]

        outbound = self.choose_door(shelf, to_inventory=True)
        if outbound is None or math.isinf(outbound["cost"]):
            self.get_logger().error("No route to the shelf.")
            return False

        self.get_logger().info(
            f"Selected {outbound['name']} for {shelf_name}."
        )

        if not self.navigate_through_door(
            outbound,
            shelf,
            shelf_name,
        ):
            return False

        self.get_logger().info(
            f"Tote picked from {shelf_name}."
        )

        inbound = self.choose_door(packout, to_inventory=False)
        if inbound is None or math.isinf(inbound["cost"]):
            self.get_logger().error("No route to Packout.")
            return False

        self.get_logger().info(
            f"Selected {inbound['name']} for Packout."
        )

        if not self.navigate_through_door(
            inbound,
            packout,
            bay_name,
        ):
            return False

        self.get_logger().info(
            f"Tote dropped at {bay_name}."
        )

        return self.navigate_to(
            SERVICE_CHARGE_BAY,
            "service charge bay",
        )

    def run(self):
        self.waitUntilNav2Active()

        if not self.wait_for_pose():
            self.get_logger().error(
                "Service AMCL pose was not received."
            )
            return

        self.publish_state(
            "idle",
            "Service worker is ready.",
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.pending_task is None:
                continue

            shelf_name, bay_name = self.pending_task
            self.pending_task = None
            self.busy = True

            self.publish_state(
                "busy",
                f"Service is packing from {shelf_name}.",
            )

            if self.execute_task(shelf_name, bay_name):
                result = "Service Packout task completed."
                self.publish_state("completed", result)
            else:
                result = "Service Packout task failed."
                self.publish_state("failed", result)

            self.busy = False
            self.publish_state("idle", f"{result} Service worker is ready.")


def main():
    rclpy.init()
    mission = WarehouseOperationsService()

    try:
        mission.run()
    finally:
        mission.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
