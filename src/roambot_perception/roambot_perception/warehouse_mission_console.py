import json
from datetime import datetime
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String


SHELF_ORDER = ("shelf_1", "shelf_2", "shelf_3")
DOOR_ORDER = ("lower_door", "upper_door")
BUSY_SERVICE_STATES = {
    "queued",
    "travelling_to_shelf",
    "at_shelf",
    "returning",
}
BUSY_SCAN_STATES = {"queued", "scanning", "returning"}


class WarehouseMissionConsole:
    def __init__(self):
        rclpy.init()
        self.node = Node("warehouse_mission_console")
        self.shelves = {}
        self.cards = {}
        self.traffic_cards = {}
        self.events_history = []
        self.scan_state = "idle"
        self.service_state = "unknown"
        self.service_shelf = None

        retained_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.inventory_request_pub = self.node.create_publisher(
            String,
            "/warehouse/inventory_request",
            10,
        )
        self.inspection_request_pub = self.node.create_publisher(
            String,
            "/warehouse/inspection_request",
            10,
        )
        self.node.create_subscription(
            String,
            "/warehouse/inventory_snapshot",
            self.snapshot_callback,
            retained_qos,
        )
        self.node.create_subscription(
            String,
            "/warehouse/inspection_status",
            self.inspection_status_callback,
            retained_qos,
        )
        self.node.create_subscription(
            String,
            "/warehouse/service_state",
            self.service_state_callback,
            retained_qos,
        )
        self.node.create_subscription(
            String,
            "/warehouse/traffic_snapshot",
            self.traffic_snapshot_callback,
            retained_qos,
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
        self.root.geometry("980x720")
        self.root.minsize(820, 580)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.build_ui()
        self.append_event(
            "Console ready. Waiting for warehouse status.",
            "System",
        )
        self.root.after(50, self.spin_ros)

    def build_ui(self):
        style = ttk.Style()
        style.configure("Title.TLabel", font=("", 18, "bold"))
        style.configure("SectionTitle.TLabel", font=("", 11, "bold"))
        style.configure("Status.TLabel", font=("", 10, "bold"))

        container = ttk.Frame(self.root, padding=16)
        container.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(container)
        header.pack(fill=tk.X)

        title = ttk.Frame(header)
        title.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(
            title,
            text="RoamBot Mission Console",
            style="Title.TLabel",
        ).pack(anchor=tk.W)

        self.connection_var = tk.StringVar(
            value="Warehouse status: waiting for Scout scan"
        )
        ttk.Label(
            title,
            textvariable=self.connection_var,
        ).pack(anchor=tk.W, pady=(2, 0))

        self.scan_button = ttk.Button(
            header,
            text="Start Inspection",
            command=self.request_inspection,
        )
        self.scan_button.pack(side=tk.RIGHT, padx=(12, 0), pady=4)

        robots_frame = ttk.LabelFrame(
            container,
            text="Robot status",
            padding=10,
        )
        robots_frame.pack(fill=tk.X, pady=(14, 8))
        robots_frame.columnconfigure(0, weight=1)
        robots_frame.columnconfigure(1, weight=1)

        self.scout_state_var = tk.StringVar(value="Scout: waiting")
        self.scout_detail_var = tk.StringVar(
            value="Awaiting an inspection request."
        )
        self.service_state_var = tk.StringVar(value="Service: waiting")
        self.service_detail_var = tk.StringVar(
            value="Awaiting warehouse readiness."
        )

        self.build_robot_card(
            robots_frame,
            0,
            "Scout",
            self.scout_state_var,
            self.scout_detail_var,
        )
        self.build_robot_card(
            robots_frame,
            1,
            "Service",
            self.service_state_var,
            self.service_detail_var,
        )

        shelves_frame = ttk.Frame(container)
        shelves_frame.pack(fill=tk.X, pady=(8, 0))

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
                wraplength=220,
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
        traffic_frame.columnconfigure(0, weight=1)
        traffic_frame.columnconfigure(1, weight=1)

        for column, door_name in enumerate(DOOR_ORDER):
            state_var = tk.StringVar(value="Free")
            card = ttk.Frame(traffic_frame)
            card.grid(
                row=0,
                column=column,
                sticky="ew",
                padx=(0 if column == 0 else 12, 0),
            )
            ttk.Label(
                card,
                text=door_name.replace("_", " ").title(),
                style="SectionTitle.TLabel",
            ).pack(anchor=tk.W)
            ttk.Label(
                card,
                textvariable=state_var,
                wraplength=410,
                justify=tk.LEFT,
            ).pack(anchor=tk.W, pady=(3, 0))
            self.traffic_cards[door_name] = state_var

        self.traffic_message_var = tk.StringVar(
            value="No doorway traffic events received yet."
        )
        ttk.Label(
            traffic_frame,
            textvariable=self.traffic_message_var,
            wraplength=900,
            justify=tk.LEFT,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(10, 0))

        events_frame = ttk.LabelFrame(
            container,
            text="Mission events",
            padding=8,
        )
        events_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))

        event_tools = ttk.Frame(events_frame)
        event_tools.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(event_tools, text="Show:").pack(side=tk.LEFT)

        self.event_filter = tk.StringVar(value="All")
        filter_box = ttk.Combobox(
            event_tools,
            textvariable=self.event_filter,
            values=("All", "System", "Scan", "Service", "Traffic"),
            state="readonly",
            width=12,
        )
        filter_box.pack(side=tk.LEFT, padx=(6, 0))
        filter_box.bind("<<ComboboxSelected>>", self.refresh_events)

        ttk.Button(
            event_tools,
            text="Clear Events",
            command=self.clear_events,
        ).pack(side=tk.RIGHT)

        self.events = scrolledtext.ScrolledText(
            events_frame,
            height=13,
            state=tk.DISABLED,
            wrap=tk.WORD,
        )
        self.events.pack(fill=tk.BOTH, expand=True)

    def build_robot_card(self, parent, column, name, state_var, detail_var):
        card = ttk.Frame(parent)
        card.grid(
            row=0,
            column=column,
            sticky="ew",
            padx=(0 if column == 0 else 12, 0),
        )
        ttk.Label(
            card,
            text=name,
            style="SectionTitle.TLabel",
        ).pack(anchor=tk.W)
        ttk.Label(
            card,
            textvariable=state_var,
            style="Status.TLabel",
        ).pack(anchor=tk.W, pady=(3, 2))
        ttk.Label(
            card,
            textvariable=detail_var,
            wraplength=410,
            justify=tk.LEFT,
        ).pack(anchor=tk.W)

    def snapshot_callback(self, message):
        try:
            snapshot = json.loads(message.data)
            shelves = {
                shelf["name"]: shelf
                for shelf in snapshot["shelves"]
            }
        except (json.JSONDecodeError, KeyError, TypeError):
            self.append_event(
                "Received an invalid inventory snapshot.",
                "System",
            )
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

        self.update_request_buttons()
        self.append_event("Inventory snapshot received from Scout.", "Scan")

    def inspection_status_callback(self, message):
        status = self.decode_status(message.data, "inspection")
        if status is None:
            return

        self.scan_state = status["state"]
        current_shelf = status.get("current_shelf")
        detail = status.get("message") or "No inspection detail available."

        state_text = self.format_state(self.scan_state)
        if current_shelf:
            state_text += (
                f" — {current_shelf.replace('_', ' ').title()}"
            )

        self.scout_state_var.set(f"Scout: {state_text}")
        self.scout_detail_var.set(detail)

        if self.scan_state in BUSY_SCAN_STATES:
            self.connection_var.set("Warehouse status: Scout scan in progress")
            self.scan_button.configure(state=tk.DISABLED)
        else:
            self.scan_button.configure(state=tk.NORMAL)
            self.scan_button.configure(
                text=(
                    "Rescan Inventory"
                    if self.shelves
                    else "Start Inspection"
                )
            )

        self.append_event(detail, "Scan")

    def service_state_callback(self, message):
        status = self.decode_status(message.data, "Service")
        if status is None:
            return

        self.service_state = status["state"]
        self.service_shelf = status.get("current_shelf")
        detail = status.get("message") or "No Service detail available."

        state_text = self.format_state(self.service_state)
        if self.service_shelf:
            state_text += (
                f" — {self.service_shelf.replace('_', ' ').title()}"
            )

        self.service_state_var.set(f"Service: {state_text}")
        self.service_detail_var.set(detail)
        self.update_request_buttons()
        self.append_event(detail, "Service")

    def traffic_snapshot_callback(self, message):
        try:
            snapshot = json.loads(message.data)
            doors = snapshot["doors"]
        except (json.JSONDecodeError, KeyError, TypeError):
            self.append_event(
                "Received an invalid traffic snapshot.",
                "System",
            )
            return

        for door_name in DOOR_ORDER:
            door = doors.get(door_name, {})
            owner = door.get("owner")
            queue = door.get("queue", [])

            if owner:
                state = f"In use by {owner.title()}"
            else:
                state = "Free"

            if queue:
                state += f" | Waiting: {', '.join(name.title() for name in queue)}"

            self.traffic_cards[door_name].set(state)

    def task_status_callback(self, message):
        self.append_event(message.data, "Service")

    def traffic_status_callback(self, message):
        self.traffic_message_var.set(message.data)
        self.append_event(message.data, "Traffic")

    def request_inspection(self):
        if self.scan_state in BUSY_SCAN_STATES:
            return

        if self.shelves:
            proceed = messagebox.askyesno(
                "Rescan inventory",
                "Start a new Scout inspection? "
                "The current inventory view will update when it finishes.",
                parent=self.root,
            )
            if not proceed:
                return

        request = String()
        request.data = "scan"
        self.inspection_request_pub.publish(request)
        self.scan_state = "queued"
        self.scout_state_var.set("Scout: Queued")
        self.scout_detail_var.set("Inspection request sent to Scout.")
        self.connection_var.set("Warehouse status: inspection requested")
        self.scan_button.configure(state=tk.DISABLED)
        self.append_event("Requested Scout inventory inspection.", "Scan")

    def request_service(self, shelf_name):
        if self.service_state in BUSY_SERVICE_STATES:
            self.append_event(
                "Service request blocked: Service is already busy.",
                "Service",
            )
            return

        shelf = self.shelves.get(shelf_name)
        if shelf is None or not shelf.get("confirmed", False):
            self.append_event(
                f"Service request blocked: {shelf_name} is not confirmed.",
                "Service",
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
                    f"Service request cancelled for empty {shelf_name}.",
                    "Service",
                )
                return

        request = String()
        request.data = str(shelf["tag_id"])
        self.inventory_request_pub.publish(request)

        self.service_state = "queued"
        self.service_shelf = shelf_name
        self.service_state_var.set(
            f"Service: Queued — {shelf_name.replace('_', ' ').title()}"
        )
        self.service_detail_var.set("Service request sent to coordinator.")
        self.update_request_buttons()
        self.append_event(
            f"Requested Service for {shelf_name} "
            f"(inventory id={shelf['tag_id']}).",
            "Service",
        )

    def update_request_buttons(self):
        service_is_busy = self.service_state in BUSY_SERVICE_STATES

        for shelf_name, card in self.cards.items():
            shelf = self.shelves.get(shelf_name)
            confirmed = bool(shelf and shelf.get("confirmed"))
            card["button"].configure(
                state=(
                    tk.NORMAL
                    if confirmed and not service_is_busy
                    else tk.DISABLED
                )
            )

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

    def decode_status(self, text, source):
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            self.append_event(
                f"Received invalid {source} status.",
                "System",
            )
            return None

    def format_state(self, state):
        return state.replace("_", " ").title()

    def append_event(self, text, category="System"):
        if not hasattr(self, "events"):
            return

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.events_history.append((timestamp, category, text))
        self.events_history = self.events_history[-250:]
        self.refresh_events()

    def refresh_events(self, _event=None):
        if not hasattr(self, "events"):
            return

        selected = self.event_filter.get()
        lines = [
            f"[{timestamp}] [{category}] {text}"
            for timestamp, category, text in self.events_history
            if selected == "All" or category == selected
        ]

        self.events.configure(state=tk.NORMAL)
        self.events.delete("1.0", tk.END)
        self.events.insert(tk.END, "\n".join(lines))
        if lines:
            self.events.insert(tk.END, "\n")
        self.events.see(tk.END)
        self.events.configure(state=tk.DISABLED)

    def clear_events(self):
        self.events_history.clear()
        self.refresh_events()

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
