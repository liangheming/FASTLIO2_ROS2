#!/usr/bin/env python3
"""
Manual Initialization Node Launch File (Standalone with all dependencies)

启动手动初始化优化节点，包含所有依赖（地图服务、TF、RViz）。
用于单独测试 manual_initialization 功能。

用法:
  ros2 launch manual_initialization manual_initialization_standalone.launch.py map_pcd:=/path/to/map.pcd
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

    search_radius_arg = DeclareLaunchArgument(
        'search_radius',
        default_value='2.0',
        description='Search radius in meters'
    )

    yaw_range_arg = DeclareLaunchArgument(
        'yaw_search_range_deg',
        default_value='20.0',
        description='Yaw search range in degrees'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            pkg_share, 'rviz', 'manual_initialization.rviz'
        ]),
        description='Path to RViz config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value='/home/steve/map_data/map_geoscan/103/global_map.pcd',
        description='Path to global map PCD file'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/rslidar_points',
        description='Pointcloud topic name'
    )

    # Include TF tree launch file - 统一TF管理
    localizer_pkg_share = FindPackageShare('localizer')
    tf_tree_launch_file = PathJoinSubstitution([
        localizer_pkg_share, 'tf', 'airy_tf_tree.launch.py'
    ])
    tf_tree_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_tree_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Global map server
    globalmap_server_node = Node(
        package='map_server',
        executable='globalmap_server_node',
        name='globalmap_server',
        output='screen',
        parameters=[
            {'globalmap_pcd': LaunchConfiguration('map_pcd')},
            {'downsample_resolution': 0.2},
            {'frame_id': 'map'},
            {'globalmap_topic': '/globalmap'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    # Manual Initialization Node
    manual_init_node = Node(
        package='manual_initialization',
        executable='manual_initialization_node',
        name='manual_initialization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'search_radius': LaunchConfiguration('search_radius'),
                'yaw_search_range_deg': LaunchConfiguration('yaw_search_range_deg'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
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
        search_radius_arg,
        yaw_range_arg,
        rviz_config_arg,
        use_sim_time_arg,
        map_pcd_arg,
        pointcloud_topic_arg,
        tf_tree_launch,
        globalmap_server_node,
        rviz_node,
        manual_init_node,
    ])
