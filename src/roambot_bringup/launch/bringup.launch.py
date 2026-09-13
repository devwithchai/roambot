import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find installed package folders safely.
    simulation_share = get_package_share_directory("roambot_simulation")
    navigation_share = get_package_share_directory("roambot_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    # Start Gazebo, the robot, and ROS-Gazebo bridges.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, "launch", "arena.launch.py")
        )
    )

    # Start map server, AMCL, planner, controller, and Nav2 behaviour tree.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "nav2.launch.py")
        )
    )

    # RViz is optional, but enabled by default.
    rviz_argument = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with the Nav2 configuration",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(nav2_share, "rviz", "nav2_default_view.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    # Gazebo needs a few seconds to create the robot and start /clock, /scan, TF.
    # Nav2 starts only after those simulation inputs are available.
    delayed_navigation = TimerAction(
        period=5.0,
        actions=[navigation, rviz],
    )

    return LaunchDescription([
        rviz_argument,
        simulation,
        delayed_navigation,
    ])
