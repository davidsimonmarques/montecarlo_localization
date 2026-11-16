"""Launch FAST-LIO2 odometry bridge alongside all 4 localization algorithms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for FAST-LIO2 + MCL, AMCL, MH-MCL, and MH-AMCL."""
    pkg_share = get_package_share_directory('montecarlo_localization')

    default_map = os.path.join(pkg_share, 'maps', 'map_factory.yaml')
    default_params = os.path.join(pkg_share, 'config', 'fastlio2_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'mcl_visualization.rviz')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to factory occupancy grid map YAML'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Unified YAML parameters for FAST-LIO2 bridge, laser_mapping, and 4 MCL nodes'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if /clock is published'
    )
    random_seed_arg = DeclareLaunchArgument(
        'random_seed',
        default_value='-1',
        description='Shared random seed for all 4 localization methods (-1 uses random_device)'
    )
    publish_tf_mcl_arg = DeclareLaunchArgument(
        'publish_tf_mcl',
        default_value='true',
        description='Whether Classical MCL broadcasts map -> odom TF'
    )
    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='true',
        description='Whether fastlio_bridge_node broadcasts odom -> base_footprint TF'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='2D LaserScan topic published by CoppeliaSim'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU topic published by CoppeliaSim'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic generated from FAST-LIO2 for the 4 localization nodes and controller'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    shared_seed = ParameterValue(LaunchConfiguration('random_seed'), value_type=int)

    # 1. Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time
        }]
    )

    # 2. Lifecycle Manager for Map Server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 3. Bridge Node: /scan (LaserScan) + /imu/data -> pseudo-3D PointCloud2 + odom_out_topic + TF
    fastlio_bridge_node = Node(
        package='montecarlo_localization',
        executable='fastlio_bridge_node',
        name='fastlio_bridge_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'publish_odom_tf': ParameterValue(
                    LaunchConfiguration('publish_odom_tf'), value_type=bool)
            }
        ]
    )

    # 4. FAST-LIO2 Mapping/Odometry Node (package 'fast_lio' from FAST_LIO_ROS2)
    # Remap /path -> /fastlio2/path so FAST-LIO2 never collides with FBL controller's /path
    fastlio_mapping_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/path', '/fastlio2/path')
        ]
    )

    # 5. Classical MCL Node
    mcl_node = Node(
        package='montecarlo_localization',
        executable='montecarlo_localization_node',
        name='montecarlo_localization_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf_mcl'), value_type=bool),
                'random_seed': shared_seed
            }
        ]
    )

    # 6. Adaptive MCL (AMCL) Node
    amcl_node = Node(
        package='montecarlo_localization',
        executable='amcl_node',
        name='amcl_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'publish_tf': False,
                'random_seed': shared_seed
            }
        ]
    )

    # 7. Metropolis-Hastings MCL (MH-MCL) Node
    mhmcl_node = Node(
        package='montecarlo_localization',
        executable='mhmcl_node',
        name='mhmcl_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'publish_tf': False,
                'random_seed': shared_seed
            }
        ]
    )

    # 8. Metropolis-Hastings AMCL (MH-AMCL) Node
    mhamcl_node = Node(
        package='montecarlo_localization',
        executable='mhamcl_node',
        name='mhamcl_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'publish_tf': False,
                'random_seed': shared_seed
            }
        ]
    )

    # 9. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        random_seed_arg,
        publish_tf_mcl_arg,
        publish_odom_tf_arg,
        use_rviz_arg,
        scan_topic_arg,
        imu_topic_arg,
        odom_topic_arg,
        map_server_node,
        TimerAction(period=1.0, actions=[lifecycle_manager_node]),
        fastlio_bridge_node,
        fastlio_mapping_node,
        mcl_node,
        amcl_node,
        mhmcl_node,
        mhamcl_node,
        rviz_node
    ])
