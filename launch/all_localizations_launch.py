import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('montecarlo_localization')

    # Default paths
    default_map = os.path.join(pkg_share, 'maps', 'map_factory.yaml')
    default_mcl_params = os.path.join(pkg_share, 'config', 'params.yaml')
    default_amcl_params = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    default_mhmcl_params = os.path.join(pkg_share, 'config', 'mhmcl_params.yaml')
    default_mhamcl_params = os.path.join(pkg_share, 'config', 'mhamcl_params.yaml')

    # Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to factory map yaml'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    random_seed_arg = DeclareLaunchArgument(
        'random_seed',
        default_value='-1',
        description='Shared random seed for all localization algorithms (-1 keeps random_device)'
    )
    publish_tf_mcl_arg = DeclareLaunchArgument(
        'publish_tf_mcl',
        default_value='true',
        description='Whether MCL publishes map->odom TF'
    )

    # 1. Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 2. Lifecycle Manager for Map Server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 3. Classical MCL Node
    mcl_node = Node(
        package='montecarlo_localization',
        executable='montecarlo_localization_node',
        name='montecarlo_localization_node',
        output='screen',
        parameters=[
            default_mcl_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': LaunchConfiguration('publish_tf_mcl'),
                'random_seed': ParameterValue(LaunchConfiguration('random_seed'), value_type=int)
            }
        ]
    )

    # 4. Adaptive MCL (AMCL) Node
    amcl_node = Node(
        package='montecarlo_localization',
        executable='amcl_node',
        name='amcl_node',
        output='screen',
        parameters=[
            default_amcl_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': False,  # Avoid TF clash with MCL
                'random_seed': ParameterValue(LaunchConfiguration('random_seed'), value_type=int)
            }
        ]
    )

    # 5. Metropolis-Hastings MCL (MH-MCL) Node
    mhmcl_node = Node(
        package='montecarlo_localization',
        executable='mhmcl_node',
        name='mhmcl_node',
        output='screen',
        parameters=[
            default_mhmcl_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': False,  # Avoid TF clash with MCL
                'random_seed': ParameterValue(LaunchConfiguration('random_seed'), value_type=int)
            }
        ]
    )

    # 6. Metropolis-Hastings AMCL (MH-AMCL) Node
    mhamcl_node = Node(
        package='montecarlo_localization',
        executable='mhamcl_node',
        name='mhamcl_node',
        output='screen',
        parameters=[
            default_mhamcl_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': False,  # Avoid TF clash with MCL
                'random_seed': ParameterValue(LaunchConfiguration('random_seed'), value_type=int)
            }
        ]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        random_seed_arg,
        publish_tf_mcl_arg,
        map_server_node,
        TimerAction(period=1.0, actions=[lifecycle_manager_node]),
        mcl_node,
        amcl_node,
        mhmcl_node,
        mhamcl_node
    ])
