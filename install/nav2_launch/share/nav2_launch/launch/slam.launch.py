
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('nav2_launch'),  # Remplacez par votre nom de package
        'config',
        'slam_params.yaml'
    )

    return LaunchDescription([

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],  # Charge le fichier YAML
            # remappings=[
            #     ('scan', '/base_scan'),
            #     ('odom', '/odom_combined'),
            #     # ajoute ici d’autres remaps si besoin
            # ]
        ),

        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=['/home/samar/takwa_ws/src/nav2_launch/config/amcl_map.yaml']
        # ),
       
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{'autostart': True},
        #                 {'node_names': [
        #                     'amcl',
        #                 ]}]
        # )
    ])
