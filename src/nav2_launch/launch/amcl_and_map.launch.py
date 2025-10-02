
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=['/home/samar/takwa_ws/src/nav2_launch/config/amcl_map.yaml']
        ),
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=['/home/samar/takwa_ws/src/nav2_launch/config/nav2_params.yaml']
        # ),
        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=['/home/samar/takwa_ws/src/nav2_launch/config/nav2_params.yaml']
        # ),
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=['/home/samar/takwa_ws/src/nav2_launch/config/nav2_params.yaml']
        # ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['/home/samar/takwa_ws/src/nav2_launch/config/amcl_map.yaml']
        ),
       
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': [
                            'map_server',
                            'amcl',
                            # 'planner_server',
                            # 'controller_server',
                            # 'bt_navigator',
                        ]}]
        )
    ])
