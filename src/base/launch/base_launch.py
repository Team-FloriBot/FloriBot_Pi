from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_share = get_package_share_directory("base")
    phidgets_share = get_package_share_directory("phidgets_high_speed_encoder")

    hardware_params = os.path.join(base_share, "params", "hardware_params.yaml")
    kinematics_params = os.path.join(base_share, "params", "kinematics_params.yaml")

    phidgets_launch = os.path.join(phidgets_share, "launch", "high_speed_encoder-launch.py")

    phidgets_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(phidgets_launch),
        launch_arguments={
            "joint0_tick2rad": "1.0",
            "joint1_tick2rad": "1.0",
            "joint2_tick2rad": "1.0",
            "joint3_tick2rad": "1.0",
        }.items()
    )

    kinematics_kwargs = {
        "package": "base",
        "executable": "kinematics_node",
        "name": "kinematics_node",
        "output": "screen",
        "parameters": [kinematics_params],
    }

    return LaunchDescription([
        phidgets_node,

        Node(
            package="base",
            executable="hardware_node",
            name="hardware_node",
            output="screen",
            parameters=[hardware_params],
        ),

        Node(**kinematics_kwargs),
    ])
