import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('montecarlo_localization')

    script_path = os.path.join(pkg_share, 'scripts', 'run_bag_experiments.py')
    if not os.path.exists(script_path):
        src_script = '/home/davidsimon/ros2_ws/src/montecarlo_localization/scripts/run_bag_experiments.py'
        if os.path.exists(src_script):
            script_path = src_script

    source_bag = LaunchConfiguration('source_bag').perform(context)
    num_exp = LaunchConfiguration('num_experiments').perform(context)
    start_idx = LaunchConfiguration('start_idx').perform(context)
    bags_dir = LaunchConfiguration('bags_dir').perform(context)
    odom_source = LaunchConfiguration('odom_source').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower()
    wait_before_play = LaunchConfiguration('wait_before_play').perform(context)
    post_play_wait = LaunchConfiguration('post_play_wait').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    cmd = [
        'python3', script_path,
        '--num-experiments', num_exp,
        '--start-idx', start_idx,
        '--bags-dir', bags_dir,
        '--odom-source', odom_source,
        '--rate', rate,
        '--wait-before-play', wait_before_play,
        '--post-play-wait', post_play_wait,
        '--seed', seed,
    ]

    if source_bag:
        cmd.extend(['--source-bag', source_bag])
    if params_file:
        cmd.extend(['--params-file', params_file])
    if use_rviz in ['true', '1', 'yes']:
        cmd.append('--use-rviz')

    process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        emulate_tty=True,
    )
    return [process]


def generate_launch_description():
    default_bags_out = os.path.expanduser('~/Documents/BagsRecordings_BagReplay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source_bag',
            default_value='',
            description='Path to source rosbag2 directory (empty = latest exp_* in ~/Documents/BagsRecordings_RP)',
        ),
        DeclareLaunchArgument(
            'num_experiments',
            default_value='10',
            description='Total number of replay experiments to execute',
        ),
        DeclareLaunchArgument(
            'start_idx',
            default_value='1',
            description='Starting experiment index',
        ),
        DeclareLaunchArgument(
            'bags_dir',
            default_value=default_bags_out,
            description='Directory to store replayed experiment bags and computation metrics',
        ),
        DeclareLaunchArgument(
            'odom_source',
            default_value='fastlio2',
            description='Odometry source for particle filters: fastlio2 or wheel',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='Bag playback rate multiplier (e.g. 1.0 for real-time)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 when odom_source is fastlio2 (true/false)',
        ),
        DeclareLaunchArgument(
            'wait_before_play',
            default_value='20.0',
            description='Max seconds to wait for all 4 localization nodes to load the map',
        ),
        DeclareLaunchArgument(
            'post_play_wait',
            default_value='3.0',
            description='Extra seconds to record after bag playback ends',
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='-1',
            description='Shared random seed (>=0) or -1 to generate a fresh random seed per experiment',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional custom YAML params file (used when odom_source=fastlio2)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
