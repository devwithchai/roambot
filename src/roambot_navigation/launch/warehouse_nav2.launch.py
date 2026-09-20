import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushROSNamespace


# Delays are measured from when warehouse_nav2.launch.py starts.
# Each robot localizes before its Nav2 navigation servers are created.
SCOUT_LOCALIZATION_DELAY = 8.0
SCOUT_NAVIGATION_DELAY = 20.0
SERVICE_LOCALIZATION_DELAY = 32.0
SERVICE_NAVIGATION_DELAY = 44.0


def include_nav2_launch(
    launch_file,
    namespace,
    params_file,
    map_file=None,
):
    arguments = {
        "namespace": namespace,
        "use_sim_time": "true",
        "autostart": "true",
        "params_file": params_file,
        "use_composition": "False",
        "use_respawn": "True",
    }

    if map_file is not None:
        arguments["map"] = map_file

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=arguments.items(),
    )


def localization_stack(
    localization_launch_file,
    namespace,
    map_file,
    params_file,
):
    return GroupAction(
        actions=[
            PushROSNamespace(namespace=namespace),
            include_nav2_launch(
                localization_launch_file,
                namespace,
                params_file,
                map_file,
            ),
        ]
    )


def navigation_stack(
    navigation_launch_file,
    namespace,
    params_file,
):
    return GroupAction(
        actions=[
            PushROSNamespace(namespace=namespace),
            include_nav2_launch(
                navigation_launch_file,
                namespace,
                params_file,
            ),
        ]
    )


def generate_launch_description():
    navigation_share = get_package_share_directory("roambot_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    launch_directory = os.path.join(nav2_share, "launch")
    localization_launch_file = os.path.join(
        launch_directory,
        "localization_launch.py",
    )
    navigation_launch_file = os.path.join(
        launch_directory,
        "navigation_launch.py",
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

    scout_localization = localization_stack(
        localization_launch_file,
        "scout",
        map_file,
        scout_params,
    )
    scout_navigation = navigation_stack(
        navigation_launch_file,
        "scout",
        scout_params,
    )
    service_localization = localization_stack(
        localization_launch_file,
        "service",
        map_file,
        service_params,
    )
    service_navigation = navigation_stack(
        navigation_launch_file,
        "service",
        service_params,
    )

    # Staging avoids starting all 28 Nav2 processes at once on the
    # 4 GB Docker allocation. A navigation stack starts only after its
    # robot's map server and AMCL have had time to become active.
    return LaunchDescription([
        TimerAction(
            period=SCOUT_LOCALIZATION_DELAY,
            actions=[
                LogInfo(
                    msg=(
                        "[RoamBot] Starting Scout localization "
                        "(map server and AMCL)."
                    )
                ),
                scout_localization,
            ],
        ),
        TimerAction(
            period=SCOUT_NAVIGATION_DELAY,
            actions=[
                LogInfo(
                    msg="[RoamBot] Starting Scout navigation."
                ),
                scout_navigation,
            ],
        ),
        TimerAction(
            period=SERVICE_LOCALIZATION_DELAY,
            actions=[
                LogInfo(
                    msg=(
                        "[RoamBot] Starting Service localization "
                        "(map server and AMCL)."
                    )
                ),
                service_localization,
            ],
        ),
        TimerAction(
            period=SERVICE_NAVIGATION_DELAY,
            actions=[
                LogInfo(
                    msg="[RoamBot] Starting Service navigation."
                ),
                service_navigation,
            ],
        ),
    ])
