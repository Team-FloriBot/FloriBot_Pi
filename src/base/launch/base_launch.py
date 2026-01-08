from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_base = get_package_share_directory('base')
    kin = os.path.join(pkg_base, 'params', 'kinematics_params.yaml')
    hw  = os.path.join(pkg_base, 'params', 'hardware_params.yaml')

    return LaunchDescription([
        # --- Hardware (Motor + Encoder -> /base/wheel_ticks4) ---
        Node(
            package='base',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
            parameters=[hw],
        ),

        # --- Kinematics (PID + Odom, publishes /base/wheel_cmd) ---
        Node(
            package='base',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kin],
        ),
    ])
