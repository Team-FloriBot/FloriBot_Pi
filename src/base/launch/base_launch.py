from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("base")

    hardware_params = os.path.join(share, "params", "hardware_params.yaml")
    kinematics_params = os.path.join(share, "params", "kinematics_params.yaml")

    # kinematics_params.yaml is optional; if it doesn't exist, launch without params.
    kinematics_node_kwargs = {
        "package": "base",
        "executable": "kinematics_node",
        "name": "kinematics_node",
        "output": "screen",
    }
    if os.path.exists(kinematics_params):
        kinematics_node_kwargs["parameters"] = [kinematics_params]

    return LaunchDescription([
        Node(**kinematics_node_kwargs),
        Node(
            package="base",
            executable="hardware_node",
            name="hardware_node",
            output="screen",
            parameters=[hardware_params],
        ),
    ])
