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

    # Start the Phidget 1047 encoder driver.
    # NOTE:
    # - By default it publishes joint0..joint3.
    # - If you prefer fl/fr/rl/rr names, set joint*_name parameters here and adjust YAML accordingly.
    phidgets_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(phidgets_launch),
        launch_arguments={
            # Make JointState.position "ticks" (so hardware_node's llround(position) works)
            "joint0_tick2rad": "1.0",
            "joint1_tick2rad": "1.0",
            "joint2_tick2rad": "1.0",
            "joint3_tick2rad": "1.0",
            # Optional naming (uncomment if you want names instead of joint0..3):
            # "joint0_name": "rl_joint",
            # "joint1_name": "rr_joint",
            # "joint2_name": "fl_joint",
            # "joint3_name": "fr_joint",
        }.items()
    )

    kinematics_kwargs = {
        "package": "base",
        "executable": "kinematics_node",
        "name": "kinematics_node",
        "output": "screen",
    }
    if os.path.exists(kinematics_params):
        kinematics_kwargs["parameters"] = [kinematics_params]

    return LaunchDescription([
        phidgets_node,
        Node(**kinematics_kwargs),
        Node(
            package="base",
            executable="hardware_node",
            name="hardware_node",
            output="screen",
            parameters=[hardware_params],
        ),
    ])
