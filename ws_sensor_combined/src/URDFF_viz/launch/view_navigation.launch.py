#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package
    pkg_urdff_viz = FindPackageShare('URDFF_viz')
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('config')
    
    # Arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    arg_rviz_config = DeclareLaunchArgument(
        'config',
        default_value='nav2.rviz',
        description='RViz config file name (in rviz/ folder)'
    )
    
    # RViz config path
    config_rviz = PathJoinSubstitution(
        [pkg_urdff_viz, 'rviz', rviz_config]
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_rviz_config)
    
    # Add nodes
    ld.add_action(rviz_node)
    
    return ld
