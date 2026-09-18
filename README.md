# RoamBot

RoamBot is a lightweight ROS 2 Jazzy warehouse simulation built with Gazebo Harmonic. It starts as a custom differential-drive robot and now supports a coordinated two-robot warehouse workflow.

## Demo snapshots

<table>
  <tr>
    <td width="50%" align="center">
      <img width="420" alt="RoamBot warehouse simulation" src="https://github.com/user-attachments/assets/b30d3729-7beb-4162-bad1-ed2a762698ed" />
      <br />
      <sub>Warehouse simulation</sub>
    </td>
    <td width="50%" align="center">
      <img width="420" alt="Scout inventory inspection" src="https://github.com/user-attachments/assets/057703f7-1a48-42fa-bee7-ea4a7b35d7d6" />
      <br />
      <sub>Scout inventory inspection</sub>
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <img width="460" alt="Coordinated two-robot warehouse workflow" src="https://github.com/user-attachments/assets/b124b820-8b90-4771-a85b-6f053366ad59" />
      <br />
      <sub>Coordinated two-robot workflow</sub>
    </td>
  </tr>
</table>

## Releases

- **v1.0.0 — Autonomous navigation:** one robot, LiDAR, SLAM, AMCL, and Nav2.
- **v2.0.0 — Multi-robot warehouse coordination:** Scout inspects inventory tags; Service fulfils requested shelf tasks; both share warehouse doorways safely.

## v2.0.0 capabilities

- Two independently namespaced robots: `/scout` and `/service`
- Scout camera-based ArUco inventory-tag inspection for shelves 1–3
- Published inventory scan report
- Inventory-ID request workflow for Service dispatch
- One controlled retry for a failed Service navigation task
- Dynamic selection of the shortest planned route through either warehouse doorway
- Shared-door traffic manager using request, grant, and release messages
- Continuous motion through an uncongested doorway; waiting only for an actual same-door conflict
- Unified one-command warehouse launch

## Workflow

1. **Scout** visits the three shelf inspection poses and confirms tags 10, 11, and 12.
2. Scout publishes its scan report and returns to its desk through the shortest planned doorway route.
3. A user requests an inventory ID.
4. **Service** accepts the matching confirmed shelf, travels there, then returns to its desk.
5. When both robots need the same doorway, the Traffic Manager grants access to one robot at a time. Robots using different doors may move concurrently.

## Project structure

```text
roambot_ws/
├── src/
│   ├── roambot_description/  # Reusable robot Xacro/URDF model
│   ├── roambot_simulation/   # Warehouse world, robot spawning, Gazebo bridges
│   ├── roambot_navigation/   # Namespaced AMCL and Nav2 configurations
│   ├── roambot_perception/   # Tag detection, missions, coordinator, traffic manager
│   └── roambot_bringup/      # Unified workflow launch
├── README.md
└── .gitignore
```

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- `nav2_bringup`
- `slam_toolbox`
- `ros_gz_sim` and `ros_gz_bridge`
- OpenCV with ArUco support

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the v2.0.0 warehouse workflow

### 1. Launch the complete backend

```bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
source install/setup.bash

ros2 launch roambot_bringup warehouse_workflow.launch.py
```

This starts Gazebo, both Nav2 stacks, inventory-tag detection, Service dispatch, the task coordinator, and the doorway Traffic Manager.

### 2. Start Scout inspection

Open another terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
source install/setup.bash

ros2 run roambot_perception inventory_scan_mission
```

Scout inspects shelves 1–3, publishes the scan report, and returns to its desk.

### 3. Request inventory

After Scout completes its scan, open another terminal and request an ID:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
source install/setup.bash

ros2 topic pub --once \
  /warehouse/inventory_request \
  std_msgs/msg/String \
  "{data: '10'}"
```

Valid IDs are `10`, `11`, and `12`. Service travels to the confirmed shelf and returns to its desk.

## Observe coordination

### Task status

```bash
ros2 topic echo /warehouse/task_status
```

### Doorway traffic status

```bash
ros2 topic echo /warehouse/traffic_status
```

A same-door handoff looks like:

```text
Traffic request: scout queued for upper_door.
Traffic grant: scout may enter upper_door.
Traffic request: service queued for upper_door.
Traffic release: scout cleared upper_door.
Traffic grant: service may enter upper_door.
```

This demonstrates mutual exclusion: Service waits only while Scout owns the same doorway.

## Core ROS topics

| Topic | Purpose |
| --- | --- |
| `/scout/inventory/detections` | Inventory tags visible to Scout |
| `/scout/inventory/scan_report` | Scout's completed shelf-confirmation report |
| `/warehouse/inventory_request` | User request containing inventory ID 10, 11, or 12 |
| `/warehouse/task_status` | Coordinator task state |
| `/warehouse/traffic_status` | Doorway queue, grant, and release events |
| `/warehouse/traffic/request` | Robot requests doorway access |
| `/warehouse/traffic/grant` | Traffic Manager grants doorway access |
| `/warehouse/traffic/release` | Robot releases a cleared doorway |

## Navigation stack

RoamBot uses:

- **AMCL** for localization on the saved warehouse map.
- **Nav2** for planning and navigation.
- **Regulated Pure Pursuit** as the local controller.
- **ros_gz_bridge** to connect Gazebo and ROS 2 communication.

## Roadmap

- **v2.1.0:** lightweight inventory-aware GUI, coloured-box quantities, and empty-shelf confirmation
- **v2.2.0:** comparison and implementation of selected robot path-planning algorithms
- **v3.0.0:** robot arms with pickup and replenishment operations
