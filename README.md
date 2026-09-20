# RoamBot

RoamBot is a lightweight ROS 2 Jazzy warehouse simulation for learning and demonstrating practical mobile-robot software. It began as a custom differential-drive robot, then grew into a two-robot workflow where **Scout** inspects inventory and **Service** fulfils a selected shelf request.

The project is designed to be understandable from the operator’s point of view as well as from the ROS 2 side: Gazebo shows the physical simulation, Nav2 plans motion, and the Mission Console explains what the robots are doing and why.

## Demo video

> **Coming soon:** this section will contain a short end-to-end demo of GUI-controlled inventory inspection, Service dispatch, doorway coordination, live logs, route decisions, and Scout’s first-person camera view.

<!-- Replace this note with the final uploaded demo-video link. -->

## Demo snapshots

<table>
  <tr>
    <td width="50%" align="center">
      <img width="380" alt="RoamBot custom robot model in RViz" src="https://github.com/user-attachments/assets/b30d3729-7beb-4162-bad1-ed2a762698ed" />
      <br />
      <sub>Custom differential-drive robot model</sub>
    </td>
    <td width="50%" align="center">
      <img width="380" alt="Scout inspecting warehouse inventory tags" src="https://github.com/user-attachments/assets/057703f7-1a48-42fa-bee7-ea4a7b35d7d6" />
      <br />
      <sub>Scout inventory inspection</sub>
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center">
      <img width="460" alt="Coordinated Scout and Service warehouse workflow" src="https://github.com/user-attachments/assets/b124b820-8b90-4771-a85b-6f053366ad59" />
      <br />
      <sub>Two-robot warehouse workflow</sub>
    </td>
  </tr>
</table>

## What RoamBot demonstrates

- Custom differential-drive robot modelling with URDF/Xacro
- ROS 2 Jazzy, Gazebo Harmonic, AMCL, Nav2, and Regulated Pure Pursuit
- Separate Scout and Service robot namespaces
- Camera-based ArUco tag confirmation for warehouse shelves
- Inventory-aware Service dispatch
- Shared-doorway traffic coordination
- Shortest-feasible-route selection using Nav2 planned-path length
- A lightweight Tkinter operator console instead of a command-heavy workflow
- Scout first-person camera viewing through RViz

## Release history

| Release | Status | Focus |
| --- | --- | --- |
| **v1.0.0** | Released | Single-robot navigation: custom RoamBot model, LiDAR, mapping/localization, and Nav2 navigation |
| **v2.0.0** | Released | Multi-robot warehouse coordination: Scout inspection, Service dispatch, doorway permissions, unified launch |
| **v2.1.0** | Current feature branch | Inventory-aware GUI, coloured-box quantities, route explanations, live mission log, export, and Scout camera view |
| **v2.2.0** | Planned | Compare and implement selected robot path-planning algorithms |
| **v3.0.0** | Planned | Robot arms, pickup/replenishment, and physical warehouse operations |

## v2.1.0 operator console

The **RoamBot Mission Console** is a small desktop GUI built with Tkinter. It sits on top of the existing ROS topics; it does not bypass Nav2, the coordinator, or the traffic manager.

| Earlier manual action | Console control or view | Why it helps |
| --- | --- | --- |
| Start Scout inspection from a terminal | **Start/Rescan Inventory** | Operators do not need to remember a ROS command |
| Publish an inventory ID manually | **Request Service** on a shelf card | Sends the correct request using the confirmed shelf information |
| Read inventory-topic output | Shelf cards with item colours, counts, status, and evidence | Stock information is visible at a glance |
| Echo task and traffic topics in separate terminals | **Live mission log**, robot cards, and doorway cards | Keeps real-time operational context in one place |
| Inspect planner output or terminal messages | Route-decision tables | Shows all four route candidates and why one was selected |
| Start RViz manually to see the camera | **View Scout Camera** | Opens Scout’s actual first-person camera stream in a compact RViz window |
| Copy terminal output for reporting | **Export Summary** | Saves the inventory, robot states, traffic state, routes, and event history as JSON |

The live log intentionally shows useful mission-level information—scan progress, Service state, traffic events, and route decisions—rather than every verbose Nav2 debug message.

### Inventory information and evidence

Each shelf card distinguishes two kinds of information:

- **Tag evidence:** Scout’s camera confirms the low-mounted ArUco shelf tag.
- **Inventory content:** the current simulated inventory catalogue supplies item colour and quantity.

Current simulated stock:

| Shelf | Tag | Status | Inventory |
| --- | --- | --- | --- |
| Shelf 1 | 10 | Empty | No boxes |
| Shelf 2 | 11 | Available | 1 red box, 2 green boxes |
| Shelf 3 | 12 | Available | 2 blue boxes, 1 green box |

This is intentionally transparent: RoamBot does not yet use computer vision to count boxes on the high shelves. The camera-visible tags identify the shelf, while the simulation catalogue provides the quantity data.

## System architecture

~~~mermaid
flowchart TD
    Operator["Mission Console"] --> ScoutMission["Scout inspection mission"]
    ScoutMission --> Detector["Camera tag detector"]
    Detector --> Snapshot["Inventory snapshot"]
    Snapshot --> Operator
    Operator --> Coordinator["Task coordinator"]
    Coordinator --> ServiceMission["Service dispatch mission"]
    ScoutMission --> Traffic["Doorway traffic manager"]
    ServiceMission --> Traffic
~~~

### Robot roles

- **Scout** visits each inspection pose, confirms shelf tags, publishes the inventory snapshot, and returns to its desk.
- **Service** accepts a user-selected confirmed shelf, travels to it, then returns to its desk.
- **Traffic Manager** grants one robot at a time permission for a shared doorway. Robots using different doors can move concurrently.
- **Mission Console** gives the operator a clear view of availability, status, traffic, routes, and mission events.

## Route selection and traffic coordination

For each cross-room journey, a robot evaluates four Nav2 route candidates:

1. **lower_door** via **inventory_left**
2. **lower_door** via **inventory_right**
3. **upper_door** via **inventory_left**
4. **upper_door** via **inventory_right**

The current policy selects the **shortest feasible Nav2 planned path**. This is not simple straight-line distance: obstacles, free space, and the global costmap affect the planned length.

After a route is selected, the Traffic Manager applies doorway mutual exclusion:

1. The robot travels to its chosen doorway clearance point.
2. It requests access.
3. If another robot owns that same door, it waits safely.
4. The manager grants access when the door is clear.
5. The robot crosses and releases the door.

The GUI exposes the candidate distances, selected route, current door owner, and queued robot so the decision is understandable rather than opaque.

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3 with Tkinter
- Nav2 (**nav2_bringup** and **nav2_simple_commander**)
- **slam_toolbox**
- **ros_gz_sim** and **ros_gz_bridge**
- OpenCV with ArUco support
- RViz2 for the Scout camera viewer

## Build

~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
colcon build --symlink-install
source install/setup.bash
~~~

## Run the GUI-controlled warehouse workflow

### 1. Start the complete backend

~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
source install/setup.bash

ros2 launch roambot_bringup warehouse_workflow.launch.py
~~~

The unified launch starts Gazebo, both namespaced Nav2 stacks, bridges, tag detection, Scout and Service mission nodes, the task coordinator, and the doorway Traffic Manager.

> The launch uses staged startup to reduce resource contention in Docker. Wait roughly one minute for all mission nodes to become ready.

### 2. Open the Mission Console

In a second terminal:

~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/roambot_ws
source install/setup.bash

ros2 run roambot_perception warehouse_mission_console
~~~

### 3. Operate the warehouse

1. Wait until the console reports that the system is ready.
2. Click **Start Inspection**.
3. Watch Scout inspect the shelves and publish the inventory snapshot.
4. Review stock, camera evidence, Scout’s return route, and traffic state.
5. Click **Request Service** for an available shelf.
6. Follow Service’s outbound and return route decisions in the console.
7. Use **View Scout Camera** to open the first-person camera image in RViz.
8. Use **Export Summary** to save a JSON mission record.

## Core ROS topics

| Topic | Purpose |
| --- | --- |
| **/warehouse/inspection_request** | Requests a Scout inspection from the GUI |
| **/warehouse/inspection_status** | Retained Scout mission state for the GUI |
| **/warehouse/inventory_snapshot** | Retained shelf tags, confirmation state, and simulated item quantities |
| **/warehouse/inventory_request** | Requests Service for an inventory ID |
| **/warehouse/service_state** | Retained Service mission state for the GUI |
| **/warehouse/task_status** | Task-coordinator status messages |
| **/warehouse/traffic_snapshot** | Retained doorway owner and queue state |
| **/warehouse/traffic_status** | Doorway request, grant, and release events |
| **/warehouse/scout_route_decision** | Scout’s route candidates and selected route |
| **/warehouse/service_route_decision** | Service’s route candidates and selected route |
| **/scout/camera/image** | Scout’s first-person camera stream |

## Project structure

~~~text
roambot_ws/
├── src/
│   ├── roambot_description/  # Robot URDF/Xacro model
│   ├── roambot_simulation/   # Gazebo warehouse world, spawns, and ROS-Gazebo bridges
│   ├── roambot_navigation/   # Namespaced AMCL and Nav2 configuration
│   ├── roambot_perception/   # Detection, missions, coordinator, traffic, GUI, inventory catalogue
│   └── roambot_bringup/      # Unified warehouse workflow launch
├── README.md
└── .gitignore
~~~

## Current limitations

- Box quantities are supplied by the simulation inventory catalogue, not yet detected visually.
- Shelf tags are deliberately placed at camera-visible height because the robots are small and shelf contents are higher.
- Only Scout currently has a simulated camera and camera bridge. Service-camera support requires a future sensor and bridge addition.
- Route selection uses shortest feasible planned path length; it does not yet predict queue wait time or optimize a multi-robot global schedule.
- RoamBot does not yet manipulate objects. Arm-based pickup and replenishment belong to v3.0.0.

## Next direction

The next technical release, **v2.2.0**, will turn route selection into an explicit learning and comparison exercise by implementing selected robot path-planning algorithms. That work can compare a custom planner against the current Nav2 baseline while retaining the practical warehouse workflow.

Contributions, feedback, and robotics-learning discussions are welcome.
