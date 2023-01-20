import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    mapper_params = os.path.join(get_package_share_directory('slam_toolbox_launch'), 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename': mapper_params}]
        ),   
    ])
