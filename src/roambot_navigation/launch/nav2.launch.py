import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navigation_share = get_package_share_directory("roambot_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    map_file = os.path.join(
        navigation_share, "maps", "roambot_arena.yaml"
    )

    params_file = os.path.join(
        navigation_share, "config", "nav2_params.yaml"
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([nav2])