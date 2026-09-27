import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String


class WarehouseOperationsCoordinator(Node):
    def __init__(self):
        super().__init__("warehouse_operations_coordinator")

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.worker_states = {"scout": "offline", "service": "offline"}
        self.active_tasks = {}

        self.inventory = {
            "shelf_1": {},
            "shelf_2": {"red": 1, "green": 2},
            "shelf_3": {"blue": 2, "green": 1},
        }

        self.status_pub = self.create_publisher(
            String, "/warehouse/operations/task_status", qos
        )
        self.inventory_pub = self.create_publisher(
            String, "/warehouse/inventory_snapshot", qos
        )
        self.scout_pub = self.create_publisher(
            String, "/warehouse/operations/scout_request", 10
        )
        self.service_pub = self.create_publisher(
            String, "/warehouse/operations/service_request", 10
        )

        self.create_subscription(
            String,
            "/warehouse/operations/task_request",
            self.task_callback,
            10,
        )
        self.create_subscription(
            String,
            "/warehouse/operations/scout_status",
            self.worker_callback,
            qos,
        )
        self.create_subscription(
            String,
            "/warehouse/operations/service_status",
            self.worker_callback,
            qos,
        )

        self.publish_inventory("Inventory system is ready.")
        self.get_logger().info(
            "Coordinator is waiting for Scout and Service workers."
        )

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_inventory(self, event):
        shelves = [
            {
                "name": shelf,
                "items": self.inventory[shelf],
                "status": "empty" if not self.inventory[shelf] else "available",
            }
            for shelf in sorted(self.inventory)
        ]

        message = json.dumps({
            "event": event,
            "shelves": shelves,
        })
        self.inventory_pub.publish(String(data=message))

    def worker_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        robot = data.get("robot")
        state = data.get("state")
        message = data.get("message", "")

        if robot not in self.worker_states:
            return

        self.worker_states[robot] = state
        self.get_logger().info(message)

        if state == "idle" and robot in self.active_tasks:
            task = self.active_tasks.pop(robot)

            if "completed" in message.lower():
                self.apply_inventory(task)
            else:
                self.publish_status(
                    f"{robot.title()} task failed. Inventory was not changed."
                )

    def task_callback(self, msg):
        parts = [part.strip() for part in msg.data.split(":")]

        if not parts:
            return

        if parts[0] == "putaway":
            self.assign_putaway(parts)
        elif parts[0] == "packout":
            self.assign_packout(parts)
        else:
            self.publish_status("Unknown task. Use putaway or packout.")

    def assign_putaway(self, parts):
        if len(parts) != 4:
            self.publish_status(
                "Use: putaway:shelf_name:color:count"
            )
            return

        _, shelf, color, count = parts

        if shelf not in self.inventory or not count.isdigit():
            self.publish_status("Invalid putaway request.")
            return

        if self.worker_states["scout"] != "idle":
            self.publish_status("Scout is not available.")
            return

        self.active_tasks["scout"] = {
            "type": "putaway",
            "shelf": shelf,
            "color": color,
            "count": int(count),
        }

        self.scout_pub.publish(String(data=shelf))
        self.publish_status(
            f"Scout assigned to put away {count} {color} item(s) at {shelf}."
        )

    def assign_packout(self, parts):
        if len(parts) not in (4, 5):
            self.publish_status(
                "Use: packout:shelf_name:color:count[:packout_a]"
            )
            return

        _, shelf, color, count, *bay = parts
        packout_bay = bay[0] if bay else "packout_a"

        if shelf not in self.inventory or not count.isdigit():
            self.publish_status("Invalid Packout request.")
            return

        count = int(count)
        available = self.inventory[shelf].get(color, 0)

        if available < count:
            self.publish_status(
                f"Not enough {color} stock at {shelf}. Available: {available}."
            )
            return

        if self.worker_states["service"] != "idle":
            self.publish_status("Service is not available.")
            return

        self.active_tasks["service"] = {
            "type": "packout",
            "shelf": shelf,
            "color": color,
            "count": count,
            "packout_bay": packout_bay,
        }

        self.service_pub.publish(
            String(data=f"{shelf}:{packout_bay}")
        )
        self.publish_status(
            f"Service assigned to pack {count} {color} item(s) from {shelf}."
        )

    def apply_inventory(self, task):
        shelf = task["shelf"]
        color = task["color"]
        count = task["count"]

        if task["type"] == "putaway":
            current = self.inventory[shelf].get(color, 0)
            self.inventory[shelf][color] = current + count
            event = f"Putaway complete: {count} {color} item(s) added to {shelf}."

        else:
            self.inventory[shelf][color] -= count

            if self.inventory[shelf][color] == 0:
                del self.inventory[shelf][color]

            event = f"Packout complete: {count} {color} item(s) removed from {shelf}."

        self.publish_status(event)
        self.publish_inventory(event)


def main():
    rclpy.init()
    node = WarehouseOperationsCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()