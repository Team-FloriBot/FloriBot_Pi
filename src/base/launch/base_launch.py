from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('base')
    kin = os.path.join(pkg, 'params', 'kinematics_params.yaml')
    hw  = os.path.join(pkg, 'params', 'hardware_params.yaml')

    return LaunchDescription([
        # --- Wheel Controller Nodes (mit korrektem Remapping) ---
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_fl',
            output='screen',
            remappings=[
                ('/wheel_controller/state', '/wheel_controller_fl/state'),
                ('/wheel_cmd', '/base/wheel_cmd')
            ],
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_fr',
            output='screen',
            remappings=[
                ('/wheel_controller/state', '/wheel_controller_fr/state'),
                ('/wheel_cmd', '/base/wheel_cmd')
            ],
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_rl',
            output='screen',
            remappings=[
                ('/wheel_controller/state', '/wheel_controller_rl/state'),
                ('/wheel_cmd', '/base/wheel_cmd')
            ],
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_rr',
            output='screen',
            remappings=[
                ('/wheel_controller/state', '/wheel_controller_rr/state'),
                ('/wheel_cmd', '/base/wheel_cmd')
            ],
        ),

        # --- Sensor Aggregation ---
        Node(
            package='base',
            executable='sensor_aggregator_node',
            name='sensor_aggregator_node',
            output='screen',
        ),

        # --- Kinematics ---
        Node(
            package='base',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kin],
        ),

        # --- Hardware ---
        Node(
            package='base',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
            parameters=[hw],
        ),
    ])
