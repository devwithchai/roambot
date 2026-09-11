from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_share = get_package_share_directory(
        "roambot_description"
    )

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "roambot.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",

        parameters=[
            {
                "robot_description": robot_description,
            }
        ],

        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",

        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",

        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )