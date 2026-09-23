import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    navigation_share = get_package_share_directory(
        "roambot_navigation"
    )
    slam_share = get_package_share_directory("slam_toolbox")

    slam_params = os.path.join(
        navigation_share,
        "config",
        "warehouse_slam_toolbox.yaml",
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_share,
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src="/tf", dst="/scout/tf"),
                SetRemap(
                    src="/tf_static",
                    dst="/scout/tf_static",
                ),
                slam_launch,
            ]
        )
    ])
