import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


MISSION_NODES_DELAY = 38.0


def generate_launch_description():
    simulation_share = get_package_share_directory("roambot_simulation")
    navigation_share = get_package_share_directory("roambot_navigation")

    # 1. Start Gazebo, both robots, bridges, TF, LiDAR, and Scout camera.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                "launch",
                "warehouse_two_robots.launch.py",
            )
        )
    )

    # 2. Start both independent Nav2 stacks in a staggered sequence.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                "launch",
                "warehouse_nav2.launch.py",
            )
        )
    )

    # 3. Scout reads its camera and publishes detected inventory IDs.
    detector = Node(
        package="roambot_perception",
        executable="inventory_tag_detector",
        output="screen",
    )

    # 4. Scout waits for a GUI inspection request.
    scanner = Node(
        package="roambot_perception",
        executable="inventory_scan_mission",
        parameters=[{"auto_start": False}],
        output="screen",
    )

    # 5. Service receives shelf-dispatch commands and navigates there.
    service_dispatcher = Node(
        package="roambot_perception",
        executable="service_dispatch_mission",
        output="screen",
    )

    # 6. Coordinator converts inventory-ID requests into Service tasks.
    coordinator = Node(
        package="roambot_perception",
        executable="warehouse_task_coordinator",
        output="screen",
    )

    # 7. Grants one robot at a time access to a shared doorway.
    traffic_manager = Node(
        package="roambot_perception",
        executable="doorway_traffic_manager",
        output="screen",
    )

    # Gazebo starts first. warehouse_nav2.launch.py then starts Scout and
    # Service Nav2 separately. Start perception only after both stacks have
    # had time to create their lifecycle services.
    delayed_navigation = TimerAction(
        period=4.0,
        actions=[navigation],
    )
    delayed_mission_nodes = TimerAction(
        period=MISSION_NODES_DELAY,
        actions=[
            detector,
            scanner,
            service_dispatcher,
            coordinator,
            traffic_manager,
        ],
    )

    return LaunchDescription([
        simulation,
        delayed_navigation,
        delayed_mission_nodes,
    ])
