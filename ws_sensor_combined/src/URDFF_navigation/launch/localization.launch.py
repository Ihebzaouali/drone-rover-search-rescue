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
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_urdff_nav, 'maps', 'classroom_map.yaml'),
        description='Full path to map yaml file'
    )
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_urdff_nav, 'config', 'localization.yaml'),
        description='Full path to localization params file (AMCL + map_server config)'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup localization'
    )
    
    # Configure params with use_sim_time
    # Adding more explicit rewrites if the nested structure is being stubborn
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    # Launch localization (AMCL + Map Server)
    localization_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_autostart,
        localization_bringup
    ])
