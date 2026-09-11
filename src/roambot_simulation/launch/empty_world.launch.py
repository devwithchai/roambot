from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    simulation_share = get_package_share_directory("roambot_simulation")
    description_share = get_package_share_directory("roambot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_file = simulation_share + "/worlds/empty_world.sdf"
    xacro_file = description_share + "/urdf/roambot.urdf.xacro"
    bridge_config = simulation_share + "/config/bridge.yaml"
    
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description,
        }],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ros_gz_sim_share + "/launch/gz_sim.launch.py"
        ),
        launch_arguments={
            "gz_args": "-r -v 3 " + world_file,
        }.items(),
    )

    spawn_roambot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-name", "roambot",
            "-string", Command(["xacro ", xacro_file]),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.08",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_roambot]),
    ])