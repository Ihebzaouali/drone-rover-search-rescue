import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_slam = get_package_share_directory('URDFF_navigation')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_parms_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_slam_params = DeclareLaunchArgument(
        'slam_parms_file',
        default_value=os.path.join(pkg_slam, 'config', 'slam_parms.yaml'),
        description='Full path to slam_toolbox parms'
    )
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        slam_toolbox_node,
    ])
