"""Shared map coordinates for warehouse doorway traffic routes."""

# Two safe parking positions near the service-room desk.
SCOUT_DESK_BAY = {
    "x": -3.014,
    "y": -0.308,
    "yaw": 0.0,
}

SERVICE_DESK_BAY = {
    "x": -2.813,
    "y": -3.571,
    "yaw": 0.0,
}

# Doorway crossing points in the map frame.
# Each door has a service-room point and two inventory-room clearance points.
DOORWAYS = {
    "lower_door": {
        "service_side": {
            "x": 0.322,
            "y": -0.459,
            "yaw": 0.0,
        },
        "inventory_left": {
            "x": 2.102,
            "y": -0.219,
            "yaw": 0.0,
        },
        "inventory_right": {
            "x": 2.045,
            "y": -0.905,
            "yaw": 0.0,
        },
    },
    "upper_door": {
        "service_side": {
            "x": 0.357,
            "y": -2.906,
            "yaw": 0.0,
        },
        "inventory_left": {
            "x": 2.045,
            "y": -2.055,
            "yaw": 0.0,
        },
        "inventory_right": {
            "x": 2.099,
            "y": -3.575,
            "yaw": 0.0,
        },
    },
}