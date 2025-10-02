from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Chemins vers les fichiers de lancement
    tbot_launch_file = os.path.join(
        get_package_share_directory('tbot_node'),
        'launch',
        'pibot_launch.yaml'
    )

    amcl_launch_file = os.path.join(
        get_package_share_directory('nav2_launch'),
        'launch',
        'amcl_and_map.launch.py'
    )

    nav_launch_file = os.path.join(
        get_package_share_directory('nav2_launch'),
        'launch',
        'navigation_launch.py'
    )

    # Lancement des launch files
    tbot_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(tbot_launch_file)  # YAML → AnyLaunch
    )

    amcl_and_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file)  # Python → PythonLaunch
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file)
    )

    # Exécution des noeuds (ros2 run)
    follow_waypoints = ExecuteProcess(
        cmd=['ros2', 'run', 'follow_waypoints', 'follow_waypoints'],
        output='screen'
    )

    rviz2 = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2'],
        output='screen'
    )

    mqtt_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mon_package', 'mqtt_subscriber_node'],
        output='screen'
    )

    influxdb_logger_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mon_package', 'influxdb_logger_node'],
        output='screen'
    )

    anomaly_detector_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mon_package', 'anomaly_detector_node'],
        output='screen'
    )

    return LaunchDescription([
        tbot_launch,
        amcl_and_map_launch,
        navigation_launch,
        follow_waypoints,
        rviz2,
        mqtt_node,
        influxdb_logger_node,
        anomaly_detector_node,
    ])
