import json
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

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
    "waiting_for_door",
}
BUSY_SCAN_STATES = {
    "queued",
    "scanning",
    "returning",
    "waiting_for_door",
}
ITEM_COLORS = {
    "red": "#c7433e",
    "green": "#32825d",
    "blue": "#3678b8",
}


class WarehouseMissionConsole:
    def __init__(self):
        rclpy.init()
        self.node = Node("warehouse_mission_console")
        self.shelves = {}
        self.cards = {}
        self.traffic_cards = {}
        self.route_cards = {}
        self.events_history = []
        self.route_decisions = {
            "scout": None,
            "service": None,
        }
        self.traffic_snapshot = {}
        self.last_traffic_event = ""
        self.scan_state = "waiting"
        self.service_state = "waiting"
        self.service_shelf = None
        self.readiness_seen = {
            "scout": False,
            "service": False,
            "traffic": False,
            "inventory": False,
        }

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
            "/warehouse/scout_route_decision",
            lambda message: self.route_decision_callback(
                message,
                "scout",
            ),
            retained_qos,
        )
        self.node.create_subscription(
            String,
            "/warehouse/service_route_decision",
            lambda message: self.route_decision_callback(
                message,
                "service",
            ),
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
        self.root.geometry("1120x820")
        self.root.minsize(960, 680)
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
        style.configure(
            "SectionTitle.TLabel",
            font=("", 11, "bold"),
        )
        style.configure("Status.TLabel", font=("", 10, "bold"))
        style.configure("Small.TLabel", font=("", 9))

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
            value="Warehouse status: waiting for system readiness"
        )
        ttk.Label(
            title,
            textvariable=self.connection_var,
        ).pack(anchor=tk.W, pady=(2, 0))

        self.export_button = ttk.Button(
            header,
            text="Export Summary",
            command=self.export_summary,
        )
        self.export_button.pack(side=tk.RIGHT, padx=(10, 0), pady=4)

        self.scan_button = ttk.Button(
            header,
            text="Start Inspection",
            state=tk.DISABLED,
            command=self.request_inspection,
        )
        self.scan_button.pack(side=tk.RIGHT, padx=(10, 0), pady=4)

        readiness_frame = ttk.LabelFrame(
            container,
            text="System readiness",
            padding=8,
        )
        readiness_frame.pack(fill=tk.X, pady=(12, 8))
        self.readiness_vars = {}

        for column, component in enumerate(
            ("scout", "service", "traffic", "inventory")
        ):
            readiness_frame.columnconfigure(column, weight=1)
            self.build_readiness_card(
                readiness_frame,
                column,
                component,
            )

        robots_frame = ttk.LabelFrame(
            container,
            text="Robot status",
            padding=10,
        )
        robots_frame.pack(fill=tk.X, pady=(8, 8))
        robots_frame.columnconfigure(0, weight=1)
        robots_frame.columnconfigure(1, weight=1)

        self.scout_state_var = tk.StringVar(value="Scout: Waiting")
        self.scout_detail_var = tk.StringVar(
            value="Waiting for Scout Nav2."
        )
        self.service_state_var = tk.StringVar(
            value="Service: Waiting"
        )
        self.service_detail_var = tk.StringVar(
            value="Waiting for Service Nav2."
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
            evidence_var = tk.StringVar(
                value="Evidence: awaiting Scout confirmation."
            )

            ttk.Label(card, textvariable=tag_var).pack(anchor=tk.W)
            ttk.Label(
                card,
                textvariable=status_var,
                style="Status.TLabel",
            ).pack(anchor=tk.W, pady=(8, 4))
            ttk.Label(
                card,
                textvariable=items_var,
                wraplength=230,
                justify=tk.LEFT,
            ).pack(anchor=tk.W, fill=tk.X)

            chips_frame = ttk.Frame(card)
            chips_frame.pack(
                anchor=tk.W,
                fill=tk.X,
                pady=(6, 2),
            )

            ttk.Label(
                card,
                textvariable=evidence_var,
                style="Small.TLabel",
                wraplength=230,
                justify=tk.LEFT,
            ).pack(anchor=tk.W, fill=tk.X, pady=(2, 0))

            button = ttk.Button(
                card,
                text="Request Service",
                state=tk.DISABLED,
                command=lambda name=shelf_name: self.request_service(
                    name
                ),
            )
            button.pack(anchor=tk.W, pady=(10, 0))

            self.cards[shelf_name] = {
                "tag": tag_var,
                "status": status_var,
                "items": items_var,
                "evidence": evidence_var,
                "chips": chips_frame,
                "button": button,
            }

        routes_frame = ttk.LabelFrame(
            container,
            text="Route decisions",
            padding=8,
        )
        routes_frame.pack(fill=tk.X, pady=(14, 8))
        routes_frame.columnconfigure(0, weight=1)
        routes_frame.columnconfigure(1, weight=1)

        self.build_route_card(routes_frame, 0, "scout")
        self.build_route_card(routes_frame, 1, "service")

        traffic_frame = ttk.LabelFrame(
            container,
            text="Doorway traffic",
            padding=10,
        )
        traffic_frame.pack(fill=tk.X, pady=(8, 8))
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
                wraplength=470,
                justify=tk.LEFT,
            ).pack(anchor=tk.W, pady=(3, 0))
            self.traffic_cards[door_name] = state_var

        self.traffic_message_var = tk.StringVar(
            value="No doorway traffic events received yet."
        )
        ttk.Label(
            traffic_frame,
            textvariable=self.traffic_message_var,
            wraplength=1020,
            justify=tk.LEFT,
        ).grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(10, 0),
        )

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
            values=(
                "All",
                "System",
                "Scan",
                "Service",
                "Traffic",
                "Route",
            ),
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
            height=8,
            state=tk.DISABLED,
            wrap=tk.WORD,
        )
        self.events.pack(fill=tk.BOTH, expand=True)

    def build_readiness_card(self, parent, column, component):
        title = {
            "scout": "Scout mission",
            "service": "Service mission",
            "traffic": "Traffic manager",
            "inventory": "Inventory",
        }[component]
        state_var = tk.StringVar(value="Waiting")
        self.readiness_vars[component] = state_var

        card = ttk.Frame(parent)
        card.grid(
            row=0,
            column=column,
            sticky="ew",
            padx=(0 if column == 0 else 8, 0),
        )
        ttk.Label(
            card,
            text=title,
            style="SectionTitle.TLabel",
        ).pack(anchor=tk.W)
        ttk.Label(
            card,
            textvariable=state_var,
            wraplength=240,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(2, 0))

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
            wraplength=480,
            justify=tk.LEFT,
        ).pack(anchor=tk.W)

    def build_route_card(self, parent, column, robot):
        card = ttk.Frame(parent)
        card.grid(
            row=0,
            column=column,
            sticky="nsew",
            padx=(0 if column == 0 else 12, 0),
        )

        purpose_var = tk.StringVar(
            value="No cross-room route selected yet."
        )
        explanation_var = tk.StringVar(
            value=(
                "RoamBot will compare four Nav2 path candidates "
                "before its next cross-room trip."
            )
        )

        ttk.Label(
            card,
            text=f"{robot.title()} route",
            style="SectionTitle.TLabel",
        ).pack(anchor=tk.W)
        ttk.Label(
            card,
            textvariable=purpose_var,
            style="Small.TLabel",
            wraplength=500,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(2, 4))

        tree = ttk.Treeview(
            card,
            columns=("candidate", "distance", "traffic", "decision"),
            show="headings",
            height=4,
        )
        tree.heading("candidate", text="Candidate")
        tree.heading("distance", text="Path")
        tree.heading("traffic", text="Door state")
        tree.heading("decision", text="Decision")
        tree.column("candidate", width=170, anchor=tk.W)
        tree.column("distance", width=75, anchor=tk.CENTER)
        tree.column("traffic", width=120, anchor=tk.W)
        tree.column("decision", width=90, anchor=tk.CENTER)
        tree.tag_configure("selected", background="#d7f0dc")
        tree.pack(fill=tk.X, expand=True)

        ttk.Label(
            card,
            textvariable=explanation_var,
            style="Small.TLabel",
            wraplength=500,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(5, 0))

        self.route_cards[robot] = {
            "purpose": purpose_var,
            "explanation": explanation_var,
            "tree": tree,
        }

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
        self.readiness_seen["inventory"] = True

        for shelf_name in SHELF_ORDER:
            shelf = self.shelves.get(shelf_name)
            if shelf is None:
                continue

            card = self.cards[shelf_name]
            confirmed = shelf.get("confirmed", False)
            status = shelf.get("status", "not_confirmed")
            items = shelf.get("items", [])

            card["tag"].set(
                f"Tag: {shelf.get('tag_id', 'unknown')}"
            )
            card["status"].set(
                f"Status: {status.replace('_', ' ').title()}"
            )
            card["items"].set(self.format_items(items, confirmed))
            card["evidence"].set(self.format_evidence(shelf))
            self.render_item_chips(
                card["chips"],
                items,
                confirmed,
            )

        self.update_readiness()
        self.update_request_buttons()
        self.append_event(
            "Inventory snapshot received from Scout.",
            "Scan",
        )

    def inspection_status_callback(self, message):
        status = self.decode_status(message.data, "inspection")
        if status is None:
            return

        self.scan_state = status.get("state", "unknown")
        current_shelf = status.get("current_shelf")
        detail = status.get("message") or (
            "No inspection detail available."
        )
        self.readiness_seen["scout"] = True

        state_text = self.format_state(self.scan_state)
        if current_shelf:
            state_text += (
                f" — {current_shelf.replace('_', ' ').title()}"
            )

        self.scout_state_var.set(f"Scout: {state_text}")
        self.scout_detail_var.set(detail)
        self.update_readiness()
        self.update_inspection_button()
        self.append_event(detail, "Scan")

    def service_state_callback(self, message):
        status = self.decode_status(message.data, "Service")
        if status is None:
            return

        self.service_state = status.get("state", "unknown")
        self.service_shelf = status.get("current_shelf")
        detail = status.get("message") or (
            "No Service detail available."
        )
        self.readiness_seen["service"] = True

        state_text = self.format_state(self.service_state)
        if self.service_shelf:
            state_text += (
                f" — {self.service_shelf.replace('_', ' ').title()}"
            )

        self.service_state_var.set(f"Service: {state_text}")
        self.service_detail_var.set(detail)
        self.update_readiness()
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

        self.traffic_snapshot = doors
        self.readiness_seen["traffic"] = True

        for door_name in DOOR_ORDER:
            self.traffic_cards[door_name].set(
                self.format_door_state(door_name)
            )

        self.update_readiness()
        self.update_traffic_explanation()
        self.refresh_route_cards()

    def route_decision_callback(self, message, robot):
        try:
            decision = json.loads(message.data)
            candidates = decision["candidates"]
        except (json.JSONDecodeError, KeyError, TypeError):
            self.append_event(
                f"Received an invalid {robot} route decision.",
                "System",
            )
            return

        if not isinstance(candidates, list):
            self.append_event(
                f"Received an invalid {robot} route candidate list.",
                "System",
            )
            return

        self.route_decisions[robot] = decision
        self.refresh_route_card(robot)

        purpose = self.format_route_purpose(
            decision.get("purpose", "route")
        )
        explanation = decision.get("explanation", "")
        if explanation:
            self.append_event(
                f"{robot.title()} {purpose.lower()}: {explanation}",
                "Route",
            )

    def task_status_callback(self, message):
        self.append_event(message.data, "Service")

    def traffic_status_callback(self, message):
        self.last_traffic_event = message.data
        self.update_traffic_explanation()
        self.append_event(message.data, "Traffic")

    def request_inspection(self):
        if not self.readiness_seen["scout"]:
            self.append_event(
                "Inspection request blocked: Scout Nav2 is not ready.",
                "System",
            )
            return

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
        self.scout_detail_var.set(
            "Inspection request sent to Scout."
        )
        self.update_readiness()
        self.update_inspection_button()
        self.append_event(
            "Requested Scout inventory inspection.",
            "Scan",
        )

    def request_service(self, shelf_name):
        if not self.service_is_ready():
            self.append_event(
                "Service request blocked: Service Nav2 is not ready.",
                "Service",
            )
            return

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
        self.service_detail_var.set(
            "Service request sent to coordinator."
        )
        self.update_readiness()
        self.update_request_buttons()
        self.append_event(
            f"Requested Service for {shelf_name} "
            f"(inventory id={shelf['tag_id']}).",
            "Service",
        )

    def update_inspection_button(self):
        scout_ready = (
            self.readiness_seen["scout"]
            and self.scan_state not in BUSY_SCAN_STATES
        )
        self.scan_button.configure(
            state=tk.NORMAL if scout_ready else tk.DISABLED
        )
        self.scan_button.configure(
            text=(
                "Rescan Inventory"
                if self.shelves
                else "Start Inspection"
            )
        )

    def update_request_buttons(self):
        service_is_busy = self.service_state in BUSY_SERVICE_STATES
        service_ready = self.service_is_ready()

        for shelf_name, card in self.cards.items():
            shelf = self.shelves.get(shelf_name)
            confirmed = bool(shelf and shelf.get("confirmed"))
            card["button"].configure(
                state=(
                    tk.NORMAL
                    if (
                        confirmed
                        and service_ready
                        and not service_is_busy
                    )
                    else tk.DISABLED
                )
            )

    def update_readiness(self):
        self.readiness_vars["scout"].set(
            self.describe_scout_readiness()
        )
        self.readiness_vars["service"].set(
            self.describe_service_readiness()
        )
        self.readiness_vars["traffic"].set(
            (
                "Ready"
                if self.readiness_seen["traffic"]
                else "Waiting for traffic manager"
            )
        )
        self.readiness_vars["inventory"].set(
            self.describe_inventory_readiness()
        )

        if self.scan_state in BUSY_SCAN_STATES:
            self.connection_var.set(
                "Warehouse status: Scout scan in progress"
            )
        elif self.shelves:
            self.connection_var.set(
                "Warehouse status: Scout scan complete"
            )
        elif (
            self.readiness_seen["scout"]
            and self.readiness_seen["service"]
            and self.readiness_seen["traffic"]
        ):
            self.connection_var.set(
                "Warehouse status: ready for inspection"
            )
        else:
            self.connection_var.set(
                "Warehouse status: waiting for system readiness"
            )

    def describe_scout_readiness(self):
        if not self.readiness_seen["scout"]:
            return "Waiting for Scout Nav2"

        if self.scan_state in BUSY_SCAN_STATES:
            return "Busy"
        if self.scan_state == "failed":
            return "Needs attention"
        return "Ready"

    def describe_service_readiness(self):
        if not self.readiness_seen["service"]:
            return "Waiting for Service Nav2"

        if self.service_state in BUSY_SERVICE_STATES:
            return "Busy"
        if self.service_state == "failed":
            return "Needs attention"
        if self.service_state == "idle":
            return "Ready"
        return self.format_state(self.service_state)

    def describe_inventory_readiness(self):
        if self.shelves:
            return "Scan complete"
        if self.scan_state in BUSY_SCAN_STATES:
            return "Scanning"
        return "Scan required"

    def service_is_ready(self):
        return (
            self.readiness_seen["service"]
            and self.service_state == "idle"
        )

    def render_item_chips(self, parent, items, confirmed):
        for widget in parent.winfo_children():
            widget.destroy()

        if not confirmed:
            ttk.Label(
                parent,
                text="Waiting for tag confirmation",
                style="Small.TLabel",
            ).pack(anchor=tk.W)
            return

        if not items:
            ttk.Label(
                parent,
                text="No boxes",
                style="Small.TLabel",
            ).pack(anchor=tk.W)
            return

        for item in items:
            color_name = item.get("color", "unknown").lower()
            quantity = item.get("quantity", "?")
            background = ITEM_COLORS.get(color_name, "#667085")
            chip = tk.Label(
                parent,
                text=f"{color_name.title()} × {quantity}",
                background=background,
                foreground="white",
                padx=6,
                pady=2,
                font=("", 9, "bold"),
            )
            chip.pack(side=tk.LEFT, padx=(0, 5))

    def format_items(self, items, confirmed):
        if not confirmed:
            return "Inventory: not confirmed by Scout"

        if not items:
            return "Inventory: empty"

        summary = ", ".join(
            f"{item.get('quantity', '?')} {item.get('color', 'unknown')}"
            for item in items
        )
        return f"Inventory: {summary}"

    def format_evidence(self, shelf):
        if not shelf.get("confirmed", False):
            return (
                "Evidence: awaiting Scout camera tag confirmation."
            )

        tag_source = shelf.get("tag_evidence", "scout_camera")
        inventory_source = shelf.get(
            "inventory_source",
            "simulation_catalog",
        )

        tag_text = (
            "Scout camera"
            if tag_source == "scout_camera"
            else tag_source.replace("_", " ")
        )
        inventory_text = inventory_source.replace("_", " ")

        return (
            f"Evidence: tag confirmed by {tag_text}; "
            f"counts from {inventory_text}."
        )

    def format_door_state(self, door_name):
        door = self.traffic_snapshot.get(door_name, {})
        owner = door.get("owner")
        queue = door.get("queue", [])

        if owner:
            state = f"In use by {owner.title()}"
        else:
            state = "Free"

        if queue:
            waiting = ", ".join(
                robot.title() for robot in queue
            )
            state += f" | Waiting: {waiting}"

        return state

    def update_traffic_explanation(self):
        waiting_messages = []
        active_messages = []

        for door_name in DOOR_ORDER:
            door = self.traffic_snapshot.get(door_name, {})
            owner = door.get("owner")
            queue = door.get("queue", [])

            if owner and queue:
                waiting_robots = ", ".join(
                    robot.title() for robot in queue
                )
                waiting_messages.append(
                    f"{waiting_robots} is waiting for "
                    f"{owner.title()} to clear {door_name}."
                )
            elif owner:
                active_messages.append(
                    f"{owner.title()} is crossing {door_name}."
                )

        if waiting_messages:
            self.traffic_message_var.set(
                " ".join(waiting_messages)
            )
        elif active_messages:
            self.traffic_message_var.set(
                " ".join(active_messages)
            )
        elif self.last_traffic_event:
            self.traffic_message_var.set(
                self.last_traffic_event
            )
        else:
            self.traffic_message_var.set(
                "Both doorways are free."
            )

    def refresh_route_cards(self):
        for robot in self.route_cards:
            self.refresh_route_card(robot)

    def refresh_route_card(self, robot):
        card = self.route_cards[robot]
        tree = card["tree"]

        for item in tree.get_children():
            tree.delete(item)

        decision = self.route_decisions.get(robot)
        if decision is None:
            card["purpose"].set(
                "No cross-room route selected yet."
            )
            card["explanation"].set(
                "RoamBot will compare four Nav2 path candidates "
                "before its next cross-room trip."
            )
            return

        purpose = self.format_route_purpose(
            decision.get("purpose", "route")
        )
        selected = decision.get("selected") or {}
        selected_key = (
            selected.get("door_name"),
            selected.get("clearance_name"),
        )

        card["purpose"].set(
            f"{purpose} | Policy: shortest feasible Nav2 path"
        )
        card["explanation"].set(
            decision.get(
                "explanation",
                "No route explanation was received.",
            )
        )

        for candidate in decision.get("candidates", []):
            door_name = candidate.get("door_name", "unknown")
            clearance_name = candidate.get(
                "clearance_name",
                "unknown",
            )
            candidate_key = (door_name, clearance_name)
            feasible = candidate.get("feasible", False)
            distance = candidate.get("distance_m")

            if feasible and distance is not None:
                distance_text = f"{distance:.2f} m"
                decision_text = (
                    "Selected"
                    if candidate_key == selected_key
                    else "Alternative"
                )
            else:
                distance_text = "Unreachable"
                decision_text = "Unavailable"

            tree.insert(
                "",
                tk.END,
                values=(
                    f"{door_name} via {clearance_name}",
                    distance_text,
                    self.format_door_state(door_name),
                    decision_text,
                ),
                tags=(
                    ("selected",)
                    if candidate_key == selected_key
                    else ()
                ),
            )

    def format_route_purpose(self, purpose):
        if purpose == "return_to_desk":
            return "Returning to desk"

        prefix = "outbound_to_"
        if purpose.startswith(prefix):
            shelf_name = purpose.removeprefix(prefix)
            return (
                "Travelling to "
                f"{shelf_name.replace('_', ' ').title()}"
            )

        return purpose.replace("_", " ").title()

    def decode_status(self, text, source):
        try:
            status = json.loads(text)
        except json.JSONDecodeError:
            self.append_event(
                f"Received invalid {source} status.",
                "System",
            )
            return None

        if not isinstance(status, dict):
            self.append_event(
                f"Received invalid {source} status.",
                "System",
            )
            return None

        return status

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

    def export_summary(self):
        file_name = filedialog.asksaveasfilename(
            parent=self.root,
            title="Export RoamBot mission summary",
            defaultextension=".json",
            initialfile=(
                "roambot_mission_"
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            ),
            filetypes=(("JSON files", "*.json"),),
        )

        if not file_name:
            return

        summary = {
            "exported_at": datetime.now().isoformat(
                timespec="seconds"
            ),
            "inventory": list(self.shelves.values()),
            "robots": {
                "scout": {
                    "state": self.scan_state,
                    "detail": self.scout_detail_var.get(),
                },
                "service": {
                    "state": self.service_state,
                    "current_shelf": self.service_shelf,
                    "detail": self.service_detail_var.get(),
                },
            },
            "traffic": self.traffic_snapshot,
            "route_decisions": self.route_decisions,
            "events": [
                {
                    "time": timestamp,
                    "category": category,
                    "message": text,
                }
                for timestamp, category, text in self.events_history
            ],
        }

        try:
            with open(file_name, "w", encoding="utf-8") as output_file:
                json.dump(summary, output_file, indent=2)
        except OSError as error:
            messagebox.showerror(
                "Export failed",
                f"Could not save the mission summary:\n{error}",
                parent=self.root,
            )
            return

        self.append_event(
            f"Mission summary exported to {file_name}.",
            "System",
        )
        messagebox.showinfo(
            "Export complete",
            "Mission summary saved successfully.",
            parent=self.root,
        )

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
