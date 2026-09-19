"""Static simulated inventory for the RoamBot warehouse."""

SHELF_INVENTORY = {
    "shelf_1": {
        "tag_id": 10,
        "items": [],
    },
    "shelf_2": {
        "tag_id": 11,
        "items": [
            {"color": "red", "quantity": 1},
            {"color": "green", "quantity": 2},
        ],
    },
    "shelf_3": {
        "tag_id": 12,
        "items": [
            {"color": "blue", "quantity": 2},
            {"color": "green", "quantity": 1},
        ],
    },
}
