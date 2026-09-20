import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


SCOUT_NAV2_DELAY = 8.0
SERVICE_NAV2_DELAY = 25.0


def nav2_stack(nav2_launch_file, namespace, map_file, params_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": "true",
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )


def generate_launch_description():
    navigation_share = get_package_share_directory("roambot_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    nav2_launch_file = os.path.join(
        nav2_share,
        "launch",
        "bringup_launch.py",
    )
    map_file = os.path.join(
        navigation_share,
        "maps",
        "roambot_warehouse.yaml",
    )
    scout_params = os.path.join(
        navigation_share,
        "config",
        "scout_nav2_params.yaml",
    )
    service_params = os.path.join(
        navigation_share,
        "config",
        "service_nav2_params.yaml",
    )

    scout_nav2 = nav2_stack(
        nav2_launch_file,
        "scout",
        map_file,
        scout_params,
    )
    service_nav2 = nav2_stack(
        nav2_launch_file,
        "service",
        map_file,
        service_params,
    )

    # On the 4 GB Docker allocation, activating both full Nav2 stacks at
    # once starves lifecycle services. Let Scout finish first, then Service.
    return LaunchDescription([
        TimerAction(
            period=SCOUT_NAV2_DELAY,
            actions=[scout_nav2],
        ),
        TimerAction(
            period=SERVICE_NAV2_DELAY,
            actions=[service_nav2],
        ),
    ])
