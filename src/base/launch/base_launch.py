from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('base')
    kin_params = os.path.join(pkg_share, 'params', 'kinematics_params.yaml')
    hw_params  = os.path.join(pkg_share, 'params', 'hardware_params.yaml')

    return LaunchDescription([
        Node(
            package='base',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kin_params],
        ),
        Node(
            package='base',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
            parameters=[hw_params],
        ),
    ])
