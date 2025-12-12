import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_base = get_package_share_directory('base')
    
    # Config paths
    kinematics_config = os.path.join(pkg_base, 'params', 'kinematics_params.yaml')
    wheel_config = os.path.join(pkg_base, 'params', 'wheel_params.yaml')

    # 1. Kinematics Node
    kinematics_node = Node(
        package='base',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[kinematics_config]
    )

    # 2. Wheel Controllers (4 instances)
    wheels = ['fl', 'fr', 'rl', 'rr']
    wheel_nodes = []

    for wheel_id in wheels:
        node = Node(
            package='base',
            executable='wheel_controller_node',
            name=f'wheel_controller_{wheel_id}',
            output='screen',
            parameters=[
                wheel_config,
                {'wheel_id': wheel_id} # Override parameter
            ]
        )
        wheel_nodes.append(node)

    # 3. Sensor Aggregator Node
    aggregator_node = Node(
        package='base',
        executable='sensor_aggregator_node',
        name='sensor_aggregator_node',
        output='screen'
    )

    return LaunchDescription([
        kinematics_node,
        *wheel_nodes,
        aggregator_node
    ])

