from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('base')
    kin = os.path.join(pkg, 'params', 'kinematics_params.yaml')
    hw  = os.path.join(pkg, 'params', 'hardware_params.yaml')

    # Falls ihr wheel_controller Parameterdateien habt, hier eintragen.
    # Wenn nicht, lassen wir parameters erstmal weg.
    # Beispiel:
    # wc_fl = os.path.join(pkg, 'params', 'wheel_controller_fl.yaml')

    return LaunchDescription([
        # --- Wheel Controller Nodes (müssen Publisher auf /wheel_controller_*/state sein) ---
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_fl',
            output='screen',
            # parameters=[wc_fl],
            # remappings=[('cmd_in','/...'), ('state_out','/wheel_controller_fl/state')]
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_fr',
            output='screen',
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_rl',
            output='screen',
        ),
        Node(
            package='base',
            executable='wheel_controller_node',
            name='wheel_controller_rr',
            output='screen',
        ),

        # --- Aggregation ---
        Node(
            package='base',
            executable='sensor_aggregator_node',
            name='sensor_aggregator_node',
            output='screen',
        ),

        # --- Kinematics (cmd_vel->wheel_cmd and ticks->odom) ---
        Node(
            package='base',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kin],
        ),

        # --- Hardware output ---
        Node(
            package='base',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
            parameters=[hw],
        ),
    ])
