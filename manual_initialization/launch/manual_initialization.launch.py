#!/usr/bin/env python3
"""
Manual Initialization Node Launch File (Standalone)

单独启动手动初始化节点，用于测试或调试。
注意：需要先启动 globalmap_server_node 提供 /globalmap 话题。

完整系统请使用: ros2 launch localizer localizer_dlio.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('manual_initialization')

    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'manual_initialization.yaml'
        ]),
        description='Path to manual initialization config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/rslidar_points',
        description='Pointcloud topic name'
    )

    # Manual Initialization Node
    manual_init_node = Node(
        package='manual_initialization',
        executable='manual_initialization_node',
        name='manual_initialization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('initialpose', '/initialpose'),
            ('cloud', LaunchConfiguration('pointcloud_topic')),
            ('map', '/globalmap'),
            ('refined_initialpose', '/refined_initialpose'),
            ('aligned_cloud', '/aligned_cloud'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        pointcloud_topic_arg,
        manual_init_node,
    ])
