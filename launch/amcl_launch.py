import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('montecarlo_localization')

    default_map = os.path.join(pkg_share, 'maps', 'map_factory.yaml')
    default_params = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'mcl_visualization.rviz')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to AMCL parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    start_map_server_arg = DeclareLaunchArgument(
        'start_map_server',
        default_value='true',
        description='Whether to start map_server alongside AMCL'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    # Map Server node (started if start_map_server is true)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_map_server')),
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # AMCL Node (nav2_amcl)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Lifecycle Manager when map_server is included
    lifecycle_manager_with_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Lifecycle Manager when map_server is already running separately
    lifecycle_manager_amcl_only = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['amcl']
        }]
    )

    # Delayed lifecycle activation
    timer_with_map = TimerAction(
        period=1.0,
        actions=[lifecycle_manager_with_map],
        condition=IfCondition(LaunchConfiguration('start_map_server'))
    )

    timer_amcl_only = TimerAction(
        period=1.0,
        actions=[lifecycle_manager_amcl_only],
        condition=UnlessCondition(LaunchConfiguration('start_map_server'))
    )

    # Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        start_map_server_arg,
        use_rviz_arg,
        map_server_node,
        amcl_node,
        timer_with_map,
        timer_amcl_only,
        rviz_node
    ])
