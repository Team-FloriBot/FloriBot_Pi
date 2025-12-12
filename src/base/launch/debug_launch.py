import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_base = get_package_share_directory('base')
    
    # --- 1. Launch-Argumente definieren ---
    
    # Ermöglicht das Ein-/Ausschalten von Rviz über die Kommandozeile
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Ermöglicht das Ein-/Ausschalten von Rqt über die Kommandozeile
    use_rqt = LaunchConfiguration('use_rqt', default='true')
    
    # Pfad zur Haupt-Launch-Datei
    base_launch_file = os.path.join(pkg_base, 'launch', 'base_launch.py')

    # Pfad zur Rviz-Konfigurationsdatei (falls vorhanden)
    rviz_config_path = os.path.join(pkg_base, 'rviz', 'base_config.rviz')
    # Wir prüfen nicht, ob die Datei existiert, Rviz wird sie einfach nicht laden können,
    # wenn sie fehlt, aber die Anwendung wird starten.
    
    # --- 2. Basissystem starten (Alle Nodes) ---
    
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file),
        launch_arguments={} # Keine Argumente für die base_launch.py
    )

    # --- 3. Rviz2 starten (Visualisierung) ---
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    # --- 4. RQt Plot / RQt GUI starten (Plotting/Debugging) ---
    
    # Wir starten rqt_plot, da dies besser für das Debugging der PID-Performance ist.
    # Wir konfigurieren die Topics, um die Fehler der vier Räder zu plotten.
    rqt_plot_arguments = [
        # Syntax: /topic_name/field_name
        '/wheel_controller_fl/state/velocity_error',
        '/wheel_controller_fr/state/velocity_error',
        '/wheel_controller_rl/state/velocity_error',
        '/wheel_controller_rr/state/velocity_error',
        # Optional: Setpoints für fl/fr
        # '/wheel_controller_fl/state/setpoint_velocity',
        # '/wheel_controller_fl/state/measured_velocity' 
    ]

    rqt_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        output='screen',
        arguments=rqt_plot_arguments,
        condition=launch.conditions.IfCondition(use_rqt)
    )
    
    # --- LaunchDescription erstellen ---
    
    return LaunchDescription([
        # Deklarationen
        DeclareLaunchArgument('use_rviz', default_value='true', description='Set to true to launch Rviz.'),
        DeclareLaunchArgument('use_rqt', default_value='true', description='Set to true to launch RQt Plot.'),
        
        # Aktionen
        base_system,
        rviz_node,
        rqt_node
    ])
