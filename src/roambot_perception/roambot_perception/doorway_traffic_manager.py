from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DOORS = ("lower_door", "upper_door")
ROBOTS = ("scout", "service")


class DoorwayTrafficManager(Node):
    def __init__(self):
        super().__init__("doorway_traffic_manager")

        self.owners = {door: None for door in DOORS}
        self.queues = {door: deque() for door in DOORS}

        self.grant_pub = self.create_publisher(
            String,
            "/warehouse/traffic/grant",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/warehouse/traffic_status",
            10,
        )

        self.create_subscription(
            String,
            "/warehouse/traffic/request",
            self.request_callback,
            10,
        )
        self.create_subscription(
            String,
            "/warehouse/traffic/release",
            self.release_callback,
            10,
        )

        self.get_logger().info(
            "Traffic manager ready for lower_door and upper_door."
        )

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    def parse_message(self, text):
        parts = text.strip().split(":", maxsplit=1)

        if len(parts) != 2:
            return None, None

        robot, door = parts

        if robot not in ROBOTS or door not in DOORS:
            return None, None

        return robot, door

    def grant_next_robot(self, door):
        if self.owners[door] is not None or not self.queues[door]:
            return

        robot = self.queues[door].popleft()
        self.owners[door] = robot

        grant = String()
        grant.data = f"{robot}:{door}"
        self.grant_pub.publish(grant)

        self.publish_status(
            f"Traffic grant: {robot} may enter {door}."
        )

    def request_callback(self, message):
        robot, door = self.parse_message(message.data)

        if robot is None:
            self.publish_status(
                "Invalid traffic request. Use scout:lower_door."
            )
            return

        if self.owners[door] == robot:
            return

        if robot in self.queues[door]:
            return

        self.queues[door].append(robot)

        self.publish_status(
            f"Traffic request: {robot} queued for {door}."
        )
        self.grant_next_robot(door)

    def release_callback(self, message):
        robot, door = self.parse_message(message.data)

        if robot is None:
            self.publish_status(
                "Invalid traffic release. Use scout:lower_door."
            )
            return

        if self.owners[door] != robot:
            self.publish_status(
                f"Traffic release ignored: {robot} does not own {door}."
            )
            return

        self.owners[door] = None

        self.publish_status(
            f"Traffic release: {robot} cleared {door}."
        )
        self.grant_next_robot(door)


def main():
    rclpy.init()
    node = DoorwayTrafficManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
