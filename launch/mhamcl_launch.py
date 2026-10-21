import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('montecarlo_localization')
    default_params = os.path.join(pkg_share, 'config', 'mhamcl_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to MH-AMCL parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether MH-AMCL broadcasts the map to odom TF transform'
    )

    mhamcl_node = Node(
        package='montecarlo_localization',
        executable='mhamcl_node',
        name='mhamcl_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }
        ]
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        publish_tf_arg,
        mhamcl_node
    ])
