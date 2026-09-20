import json
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String


SHELF_ORDER = ("shelf_1", "shelf_2", "shelf_3")


class WarehouseMissionConsole:
    def __init__(self):
        rclpy.init()
        self.node = Node("warehouse_mission_console")
        self.shelves = {}
        self.cards = {}

        snapshot_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.request_pub = self.node.create_publisher(
            String,
            "/warehouse/inventory_request",
            10,
        )
        self.node.create_subscription(
            String,
            "/warehouse/inventory_snapshot",
            self.snapshot_callback,
            snapshot_qos,
        )
        self.node.create_subscription(
            String,
            "/warehouse/task_status",
            self.task_status_callback,
            10,
        )
        self.node.create_subscription(
            String,
            "/warehouse/traffic_status",
            self.traffic_status_callback,
            10,
        )

        self.root = tk.Tk()
        self.root.title("RoamBot Mission Console")
        self.root.geometry("900x620")
        self.root.minsize(760, 520)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.build_ui()
        self.append_event("Console ready. Waiting for Scout inventory snapshot.")
        self.root.after(50, self.spin_ros)

    def build_ui(self):
        style = ttk.Style()
        style.configure("Title.TLabel", font=("", 18, "bold"))
        style.configure("ShelfName.TLabel", font=("", 13, "bold"))
        style.configure("Status.TLabel", font=("", 10, "bold"))

        container = ttk.Frame(self.root, padding=16)
        container.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            container,
            text="RoamBot Mission Console",
            style="Title.TLabel",
        ).pack(anchor=tk.W)

        self.connection_var = tk.StringVar(
            value="Warehouse status: waiting for Scout scan"
        )
        ttk.Label(
            container,
            textvariable=self.connection_var,
        ).pack(anchor=tk.W, pady=(2, 14))

        shelves_frame = ttk.Frame(container)
        shelves_frame.pack(fill=tk.X)

        for column, shelf_name in enumerate(SHELF_ORDER):
            card = ttk.LabelFrame(
                shelves_frame,
                text=shelf_name.replace("_", " ").title(),
                padding=12,
            )
            card.grid(
                row=0,
                column=column,
                sticky="nsew",
                padx=(0 if column == 0 else 8, 0),
            )
            shelves_frame.columnconfigure(column, weight=1)

            tag_var = tk.StringVar(value="Tag: waiting")
            status_var = tk.StringVar(value="Status: awaiting scan")
            items_var = tk.StringVar(value="Inventory: not available")

            ttk.Label(card, textvariable=tag_var).pack(anchor=tk.W)
            ttk.Label(
                card,
                textvariable=status_var,
                style="Status.TLabel",
            ).pack(anchor=tk.W, pady=(8, 4))
            ttk.Label(
                card,
                textvariable=items_var,
                wraplength=210,
                justify=tk.LEFT,
            ).pack(anchor=tk.W, fill=tk.X)

            button = ttk.Button(
                card,
                text="Request Service",
                state=tk.DISABLED,
                command=lambda name=shelf_name: self.request_service(name),
            )
            button.pack(anchor=tk.W, pady=(12, 0))

            self.cards[shelf_name] = {
                "tag": tag_var,
                "status": status_var,
                "items": items_var,
                "button": button,
            }

        traffic_frame = ttk.LabelFrame(
            container,
            text="Doorway traffic",
            padding=10,
        )
        traffic_frame.pack(fill=tk.X, pady=(16, 8))

        self.traffic_var = tk.StringVar(
            value="No doorway traffic events received yet."
        )
        ttk.Label(
            traffic_frame,
            textvariable=self.traffic_var,
            wraplength=820,
            justify=tk.LEFT,
        ).pack(anchor=tk.W)

        events_frame = ttk.LabelFrame(
            container,
            text="Mission events",
            padding=8,
        )
        events_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        self.events = scrolledtext.ScrolledText(
            events_frame,
            height=12,
            state=tk.DISABLED,
            wrap=tk.WORD,
        )
        self.events.pack(fill=tk.BOTH, expand=True)

    def snapshot_callback(self, message):
        try:
            snapshot = json.loads(message.data)
            shelves = {
                shelf["name"]: shelf
                for shelf in snapshot["shelves"]
            }
        except (json.JSONDecodeError, KeyError, TypeError):
            self.append_event("Received an invalid inventory snapshot.")
            return

        self.shelves = shelves
        self.connection_var.set("Warehouse status: Scout scan complete")

        for shelf_name in SHELF_ORDER:
            shelf = self.shelves.get(shelf_name)
            if shelf is None:
                continue

            card = self.cards[shelf_name]
            confirmed = shelf.get("confirmed", False)
            status = shelf.get("status", "not_confirmed")
            items = shelf.get("items", [])

            card["tag"].set(f"Tag: {shelf.get('tag_id', 'unknown')}")
            card["status"].set(
                f"Status: {status.replace('_', ' ').title()}"
            )
            card["items"].set(self.format_items(items, confirmed))
            card["button"].configure(
                state=tk.NORMAL if confirmed else tk.DISABLED
            )

        self.append_event("Inventory snapshot received from Scout.")

    def format_items(self, items, confirmed):
        if not confirmed:
            return "Inventory: not confirmed by Scout"

        if not items:
            return "Inventory: empty"

        summary = ", ".join(
            f"{item['quantity']} {item['color']}"
            for item in items
        )
        return f"Inventory: {summary}"

    def task_status_callback(self, message):
        self.append_event(message.data)

    def traffic_status_callback(self, message):
        self.traffic_var.set(message.data)
        self.append_event(message.data)

    def request_service(self, shelf_name):
        shelf = self.shelves.get(shelf_name)
        if shelf is None or not shelf.get("confirmed", False):
            self.append_event(
                f"Service request blocked: {shelf_name} is not confirmed."
            )
            return

        if shelf.get("status") == "empty":
            proceed = messagebox.askyesno(
                "Empty shelf",
                f"{shelf_name.replace('_', ' ').title()} is empty. "
                "Do you still want Service to go there?",
                parent=self.root,
            )
            if not proceed:
                self.append_event(
                    f"Service request cancelled for empty {shelf_name}."
                )
                return

        request = String()
        request.data = str(shelf["tag_id"])
        self.request_pub.publish(request)
        self.append_event(
            f"Requested Service for {shelf_name} "
            f"(inventory id={shelf['tag_id']})."
        )

    def append_event(self, text):
        if not hasattr(self, "events"):
            return

        self.events.configure(state=tk.NORMAL)
        self.events.insert(tk.END, f"{text}\n")
        self.events.see(tk.END)
        self.events.configure(state=tk.DISABLED)

    def spin_ros(self):
        if not rclpy.ok():
            return

        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(50, self.spin_ros)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    WarehouseMissionConsole().run()


if __name__ == "__main__":
    main()
