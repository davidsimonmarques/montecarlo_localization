import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('montecarlo_localization')

    # Default paths
    default_map = os.path.join(pkg_share, 'maps', 'map_factory.yaml')
    default_nav_params = os.path.join(pkg_share, 'config', 'navigation_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'mcl_visualization.rviz')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to factory map YAML file'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav_params,
        description='Path to navigation parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='rrt_star',
        description='Path planning algorithm: rrt_star or wavefront'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz2 alongside navigation'
    )

    start_map_server_arg = DeclareLaunchArgument(
        'start_map_server',
        default_value='true',
        description='Whether to start map_server (set to false if map_server is already running in another terminal)'
    )

    fake_localization_arg = DeclareLaunchArgument(
        'fake_localization',
        default_value='false',
        description='Publish static map->odom transform (set to true if testing navigation WITHOUT running MCL)'
    )

    # 1. Map Server - Imports the 25m x 25m factory occupancy grid
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_map_server')),
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False
        }]
    )

    # 2. Lifecycle Manager - Automatically configures and activates map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_map_server')),
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 3. Navigation Algorithm Node (RRT* - Default selected algorithm)
    rrt_star_node = Node(
        package='rrt_star_navigation2',
        executable='rrt_star_node2',
        name='rrt_star_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('planner'), "' == 'rrt_star'"])
        ),
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 4. Alternative Navigation Node (Wavefront Planner)
    wavefront_node = Node(
        package='wavefront_navigation',
        executable='wavefront_node',
        name='wavefront_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('planner'), "' == 'wavefront'"])
        ),
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 5. Goal Relay Node: Converts RViz /goal_pose (PoseStamped) -> /goal_position (Point)
    # Allows setting goals via RViz "2D Goal Pose" button or via CLI /goal_position
    goal_relay_node = Node(
        package='montecarlo_localization',
        executable='goal_relay_node',
        name='goal_relay_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 6. Optional RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 7. Optional Static map->odom TF (Active ONLY when fake_localization:=true, for testing without MCL)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=IfCondition(LaunchConfiguration('fake_localization')),
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        planner_arg,
        use_rviz_arg,
        start_map_server_arg,
        fake_localization_arg,
        map_server_node,
        TimerAction(period=1.0, actions=[lifecycle_manager_node]),
        rrt_star_node,
        wavefront_node,
        goal_relay_node,
        static_tf_node,
        rviz_node
    ])
