import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Packages
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_urdff_nav = get_package_share_directory('URDFF_navigation')
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_urdff_nav, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params file'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup nav2 stack'
    )
    
    # Configure params with use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    # Launch ONLY the navigation stack (no localization, no map server)
    # This includes: controller, planner, behavior server, bt_navigator, smoother, waypoint_follower
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_autostart,
        nav2_bringup
    ])
