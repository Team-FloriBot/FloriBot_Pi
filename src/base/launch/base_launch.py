from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_base = get_package_share_directory('base')
    kin = os.path.join(pkg_base, 'params', 'kinematics_params.yaml')
    hw  = os.path.join(pkg_base, 'params', 'hardware_params.yaml')

    # Phidgets encoder launch (phidgets_high_speed_encoder)
    pkg_ph = get_package_share_directory('phidgets_high_speed_encoder')
    ph_launch = os.path.join(pkg_ph, 'launch', 'high_speed_encoder-launch.py')

    return LaunchDescription([
        # --- Phidget High Speed Encoder (1047) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ph_launch),
        ),

        # --- Wheel Controller Nodes ---
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_fl',
            output='screen',
            remappings=[('/wheel_commands', '/base/wheel_cmd')],
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_fr',
            output='screen',
            remappings=[('/wheel_commands', '/base/wheel_cmd')],
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_rl',
            output='screen',
            remappings=[('/wheel_commands', '/base/wheel_cmd')],
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_rr',
            output='screen',
            remappings=[('/wheel_commands', '/base/wheel_cmd')],
        ),

        # --- Sensor Aggregation ---
        Node(
            package='base',
            executable='sensor_aggregator_node',
            name='sensor_aggregator_node',
            output='screen',
            parameters=[{
                "joint_states_topic": "/joint_states"
            }],
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
