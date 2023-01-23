import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'nav2_yaml.yaml')
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_yaml.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_yaml.yaml')
    behavior_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'behavior_yaml.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'amcl_config.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters.yaml')
    waypoint_follower_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'waypoint_follower.yaml')
    map_file = os.path.join(get_package_share_directory('path_planner_server'), 'map', 'museum.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file},
                        {'topic_name': 'map'},
                        {'frame_id': 'map'}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower_yaml]
        ),
                        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml],
            output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     parameters=[{'use_sim_time': True}],
        #     output='screen'
        # ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_path',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        'filter_mask_server',
                                        'costmap_filter_info_server']}]
        ),      
    ])
