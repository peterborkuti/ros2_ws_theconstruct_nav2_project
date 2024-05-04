import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    shared_dir = get_package_share_directory('path_planner_server')
    config_dir = os.path.join(shared_dir, 'config') 

    transform_node = Node(
            package='path_planner_server',
            executable='transform',
            name='path_planner_transfomer',
            output='screen',
            parameters=[{'use_sim_time': True}]
    )

    walker_node = Node(
            package='path_planner_server',
            executable='walk',
            name='path_planner_transfomer',
            output='screen',
            parameters=[{'use_sim_time': True}]
    )


    nodes = [
        transform_node,
        walker_node
    ]

    return LaunchDescription(nodes)