import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('montecarlo_localization')
    
    # Path to script
    script_path = os.path.join(pkg_share, 'scripts', 'run_automated_experiments.py')
    if not os.path.exists(script_path):
        # Fallback to src directory if symlinked or running from source
        src_script = '/home/davidsimon/ros2_ws/src/montecarlo_localization/scripts/run_automated_experiments.py'
        if os.path.exists(src_script):
            script_path = src_script

    num_exp = LaunchConfiguration('num_experiments').perform(context)
    start_idx = LaunchConfiguration('start_idx').perform(context)
    bags_dir = LaunchConfiguration('bags_dir').perform(context)
    timeout = LaunchConfiguration('timeout').perform(context)
    wait_time = LaunchConfiguration('wait_before_fbl').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower()
    coppelia_bin = LaunchConfiguration('coppelia_bin').perform(context)
    scene_path = LaunchConfiguration('scene_path').perform(context)
    path_file = LaunchConfiguration('path_file').perform(context)

    cmd = [
        'python3', script_path,
        '--num-experiments', num_exp,
        '--start-idx', start_idx,
        '--bags-dir', bags_dir,
        '--timeout', timeout,
        '--wait-before-fbl', wait_time,
        '--coppelia-bin', coppelia_bin,
        '--scene-path', scene_path,
        '--path-file', path_file,
    ]

    if headless in ['false', '0', 'no']:
        cmd.append('--no-headless')
    else:
        cmd.append('--headless')

    process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        emulate_tty=True
    )

    return [process]


def generate_launch_description():
    pkg_share = get_package_share_directory('montecarlo_localization')

    default_scene = os.path.join(pkg_share, 'maps', 'simulation', 'Factory.ttt')
    default_coppelia = os.path.expanduser('~/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu22_04/coppeliaSim.sh')
    default_bags = os.path.expanduser('~/Documents/BagsRecordings_RP')
    default_path_file = os.path.expanduser(
        '~/Documents/BagsRecordings_PADE_RandomSeeds/poses_amostradas.txt'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_experiments',
            default_value='10',
            description='Total number of experiments to execute'
        ),
        DeclareLaunchArgument('start_idx', default_value='1', description='Starting experiment index (e.g. 1)'),
        DeclareLaunchArgument('bags_dir', default_value=default_bags, description='Directory to store the rosbag recordings'),
        DeclareLaunchArgument('timeout', default_value='360.0', description='Timeout per experiment in seconds'),
        DeclareLaunchArgument('wait_before_fbl', default_value='5.0', description='Wait time before starting FBL controller in seconds'),
        DeclareLaunchArgument('headless', default_value='false', description='Headless mode via xvfb if available (true/false)'),
        DeclareLaunchArgument('coppelia_bin', default_value=default_coppelia, description='Path to coppeliaSim.sh script'),
        DeclareLaunchArgument('scene_path', default_value=default_scene, description='Path to Factory.ttt scene'),
        DeclareLaunchArgument('path_file', default_value=default_path_file, description='Path to poses_amostradas.txt waypoint file'),
        OpaqueFunction(function=launch_setup)
    ])
