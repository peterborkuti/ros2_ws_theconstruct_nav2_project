import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    shared_dir = get_package_share_directory('path_planner_server')
    config_dir = os.path.join(shared_dir, 'config') 

    planner_yaml = os.path.join(config_dir, 'planner_server.yaml')
    controller_yaml = os.path.join(config_dir, 'controller.yaml')
    recovery_yaml = os.path.join(config_dir, 'recovery.yaml')

    bt_navigator_yaml_raw = os.path.join(config_dir, 'bt_navigator.yaml')
    context = LaunchContext()
    param_substitutions = {
        'default_nav_to_pose_bt_xml': os.path.join(config_dir, 'behavior.xml')
    }

    configured_params = RewrittenYaml(
        source_file=bt_navigator_yaml_raw,
        root_key='',
        param_rewrites=param_substitutions
    )

    bt_navigator_yaml = configured_params.perform(context)

    print('yaml:' + str(bt_navigator_yaml))

    managed_node_names = [
        'planner_server', 'controller_server',
        'bt_navigator', 'recoveries_server'
    ]

    planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml]
    )

    bt_nav_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
    )

    recovery_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        parameters=[recovery_yaml],
        output='screen',
    )

    managed_node_names = [
        'planner_server',
        'controller_server',
        'bt_navigator',
        'recoveries_server'
    ]

    lifecycle_mgr_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': managed_node_names }]
        )


    nodes = [
        planner_node,
        controller_node,
        bt_nav_node,
        recovery_node,

        lifecycle_mgr_node,
    ]

    return LaunchDescription(nodes)