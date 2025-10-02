from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node', //pour un robot réel ( async_ pour la simulation)
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}] //on travaille en temps réel, pas avec une simulation (comme Gazebo)
        )
    ])
