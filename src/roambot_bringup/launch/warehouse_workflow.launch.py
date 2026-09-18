import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Locate installed ROS package folders.
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

    # 2. Start both independent Nav2 stacks.
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

    # 4. Service receives shelf-dispatch commands and navigates there.
    service_dispatcher = Node(
        package="roambot_perception",
        executable="service_dispatch_mission",
        output="screen",
    )

    # 5. Coordinator converts inventory-ID requests into Service tasks.
    coordinator = Node(
        package="roambot_perception",
        executable="warehouse_task_coordinator",
        output="screen",
    )

    # 6. Grants one robot at a time access to a shared doorway.
    traffic_manager = Node(
        package="roambot_perception",
        executable="doorway_traffic_manager",
        output="screen",
    )

    # Gazebo begins first. The navigation launch itself has its own
    # Scout/Service startup delays.
    delayed_navigation = TimerAction(
        period=4.0,
        actions=[navigation],
    )

    # Start mission-support nodes after the simulated robots and camera
    # have had time to appear. The nodes also wait safely for Nav2/report data.
    delayed_mission_nodes = TimerAction(
        period=16.0,
        actions=[
            detector,
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
