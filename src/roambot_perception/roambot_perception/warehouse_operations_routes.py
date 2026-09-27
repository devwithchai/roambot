"""Map-frame locations for the warehouse operations world."""

SCOUT_CHARGE_BAY = {"x": -2.507, "y": 0.060, "yaw": 0.0}
SERVICE_CHARGE_BAY = {"x": -1.242, "y": 0.042, "yaw": 0.0}

WORK_STATIONS = {
    "receiving_station": {"x": -2.008, "y": 3.342, "yaw": 0.0},
    "pallet_receiving_area": {"x": -2.615, "y": 3.173, "yaw": 0.0},
    "tote_buffer": {"x": 5.412, "y": 0.198, "yaw": 0.0},
}

PACKOUT_BAYS = {
    "packout_a": {"x": -2.018, "y": 2.750, "yaw": 0.0},
    "packout_b": {"x": -2.019, "y": 1.508, "yaw": 0.0},
}

SHELF_POSES = {
    "shelf_1": {"x": 4.069, "y": 0.027, "yaw": 0.0},
    "shelf_2": {"x": 4.069, "y": 1.933, "yaw": 0.0},
    "shelf_3": {"x": 4.097, "y": 3.779, "yaw": 0.0},
}

DOORWAYS = {
    "lower_door": {
        "operations_side": {"x": 0.584, "y": 0.673, "yaw": 0.0},
        "inventory_side": {"x": 2.232, "y": 0.638, "yaw": 0.0},
    },
    "upper_door": {
        "operations_side": {"x": 0.574, "y": 3.221, "yaw": 0.0},
        "inventory_side": {"x": 2.289, "y": 3.252, "yaw": 0.0},
    },
}
