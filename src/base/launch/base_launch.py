from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('base')
    kin = os.path.join(pkg, 'params', 'kinematics_params.yaml')
    hw  = os.path.join(pkg, 'params', 'hardware_params.yaml')

    return LaunchDescription([
        Node(
            package='base',
            executable='sensor_aggregator_node',
            name='sensor_aggregator_node',
            output='screen',
        ),
        Node(
            package='base',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kin],
        ),
        Node(
            package='base',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
            parameters=[hw],
        ),
    ])
