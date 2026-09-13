import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def robot_xacro(xacro_file, model_name):
    """Create a model-specific Xacro command."""
    return Command([
        "xacro ",
        xacro_file,
        " model_name:=",
        model_name,
        " frame_prefix:=",
        model_name,
        "/",
    ])


def generate_launch_description():
    simulation_share = get_package_share_directory("roambot_simulation")
    description_share = get_package_share_directory("roambot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(
        simulation_share,
        "worlds",
        "roambot_warehouse.sdf",
    )
    xacro_file = os.path.join(
        description_share,
        "urdf",
        "roambot.urdf.xacro",
    )

    scout_bridge_config = os.path.join(
        simulation_share,
        "config",
        "scout_bridge.yaml",
    )
    service_bridge_config = os.path.join(
        simulation_share,
        "config",
        "service_bridge.yaml",
    )

    scout_xacro = robot_xacro(xacro_file, "scout")
    service_xacro = robot_xacro(xacro_file, "service")

    scout_description = ParameterValue(
        scout_xacro,
        value_type=str,
    )
    service_description = ParameterValue(
        service_xacro,
        value_type=str,
    )

    spawn_service_argument = DeclareLaunchArgument(
        "spawn_service",
        default_value="true",
        description="Spawn the Service robot as well as the Scout robot",
    )

    service_condition = IfCondition(
        LaunchConfiguration("spawn_service")
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r -v 3 " + world_file,
        }.items(),
    )

    # Each robot-state publisher handles only its own robot's joints and frames.
    scout_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="scout",
        remappings=[("/tf","/scout/tf"), ("/tf_static", "/scout/tf_static")],
        parameters=[{
            "use_sim_time": True,
            "frame_prefix": "scout/",
            "robot_description": scout_description,
        }],
        output="screen",
    )

    service_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="service",
        remappings=[("/tf", "/service/tf"), ("/tf_static", "/service/tf_static")],
        condition=service_condition,
        parameters=[{
            "use_sim_time": True,
            "frame_prefix": "service/",
            "robot_description": service_description,
        }],
        output="screen",
    )

    scout_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": scout_bridge_config}],
        output="screen",
    )

    service_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=service_condition,
        parameters=[{"config_file": service_bridge_config}],
        output="screen",
    )

    # Yellow start zone: lower-right side, facing toward the inventory entrance.
    spawn_scout = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "warehouse",
            "-name", "scout",
            "-string", scout_xacro,
            "-x", "1.20",
            "-y", "-1.65",
            "-z", "0.08",
            "-Y", "3.14159",
        ],
        output="screen",
    )

    # Cyan start zone: upper-right side.
    spawn_service = Node(
        package="ros_gz_sim",
        executable="create",
        condition=service_condition,
        arguments=[
            "-world", "warehouse",
            "-name", "service",
            "-string", service_xacro,
            "-x", "1.20",
            "-y", "1.65",
            "-z", "0.08",
            "-Y", "3.14159",
        ],
        output="screen",
    )

    return LaunchDescription([
        spawn_service_argument,
        gazebo,
        scout_bridge,
        service_bridge,
        scout_state_publisher,
        service_state_publisher,
        TimerAction(
            period=3.0,
            actions=[spawn_scout, spawn_service],
        ),
    ])
